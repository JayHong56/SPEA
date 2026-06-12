import argparse
import csv
import socket
import struct
import time
from pathlib import Path
from typing import Optional

HOST_DEFAULT = "192.168.1.10"
PORT_DEFAULT = 6001
BYTES_PER_POINT = 16
DEFAULT_MAX_RESULT_BYTES = 64 * 1024 * 1024

# New protocol:
#   8-byte big-endian voxelize_time_us
# + 4-byte big-endian result length
# + 4-byte big-endian response flags
RESP_HDR_BYTES = 16

RESP_FLAG_DONE_FALLBACK = 1 << 0  # PL returned idle after busy, but done_latched was not set.
RESP_FLAG_WAIT_TIMEOUT = 1 << 1   # Reserved: board-side wait timeout flag, if returned.


class BoardProtocolError(RuntimeError):
    pass


class BoardBusyError(RuntimeError):
    pass


def decode_resp_flags(flags: int) -> str:
    names: list[str] = []
    if flags & RESP_FLAG_DONE_FALLBACK:
        names.append("DONE_FALLBACK")
    if flags & RESP_FLAG_WAIT_TIMEOUT:
        names.append("WAIT_TIMEOUT")

    unknown = flags & ~(RESP_FLAG_DONE_FALLBACK | RESP_FLAG_WAIT_TIMEOUT)
    if unknown:
        names.append(f"UNKNOWN_0x{unknown:08X}")

    return "|".join(names) if names else "OK"


def recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        try:
            chunk = sock.recv(n - len(buf))
        except ConnectionResetError as e:
            raise ConnectionResetError(
                f"connection reset while receiving {n} bytes, already got {len(buf)} bytes"
            ) from e
        if not chunk:
            raise RuntimeError(f"socket closed early while receiving {n} bytes, already got {len(buf)} bytes")
        buf.extend(chunk)
    return bytes(buf)


def save_as_hex_txt(path: Path, data: bytes, line_bytes: int = 16) -> None:
    with path.open("w", encoding="utf-8") as f:
        for i in range(0, len(data), line_bytes):
            chunk = data[i : i + line_bytes]
            f.write(" ".join(f"{b:02X}" for b in chunk))
            f.write("\n")


def append_error_log(
    output_dir: Path,
    frame_idx: int,
    bin_path: Path,
    input_bytes: Optional[int],
    points: Optional[int],
    voxelize_time_us: Optional[int],
    result_bytes: Optional[int],
    flags: int,
    reason: str,
    attempt: Optional[int] = None,
    extra: str = "",
) -> None:
    log_path = output_dir / "error.log"
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    flag_desc = decode_resp_flags(flags)

    with log_path.open("a", encoding="utf-8") as f:
        f.write(
            f"[{ts}] "
            f"reason={reason}, "
            f"frame_idx={frame_idx}, "
            f"input_file={bin_path}, "
            f"input_name={bin_path.name}, "
            f"input_bytes={input_bytes}, "
            f"points={points}, "
            f"voxelize_time_us={voxelize_time_us}, "
            f"result_bytes={result_bytes}, "
            f"flags=0x{flags:08X}, "
            f"flag_desc={flag_desc}"
        )
        if attempt is not None:
            f.write(f", attempt={attempt}")
        if extra:
            f.write(f", extra={extra}")
        f.write("\n")


def connect_socket(host: str, port: int, timeout: float, recv_buf: int, send_buf: int) -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    if recv_buf > 0:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, recv_buf)
    if send_buf > 0:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, send_buf)
    s.connect((host, port))
    return s


def upload_one(sock: socket.socket, bin_path: Path) -> tuple[int, int]:
    input_data = bin_path.read_bytes()
    input_len = len(input_data)

    if input_len == 0:
        raise ValueError(f"{bin_path} is empty")
    if input_len % BYTES_PER_POINT != 0:
        raise ValueError(
            f"{bin_path} length {input_len} is not aligned to {BYTES_PER_POINT} bytes/point"
        )

    # Protocol: 'I' + 4-byte big-endian input_len + raw point cloud bytes.
    sock.sendall(b"I")
    sock.sendall(struct.pack("!I", input_len))
    sock.sendall(input_data)

    ack = recv_exact(sock, 2)
    if ack != b"OK":
        # Drain a little more text if the board sent ER/BUSY/NOINPUT/BADCMD-style response.
        try:
            extra = sock.recv(16)
        except Exception:
            extra = b""
        raise BoardProtocolError(f"upload failed for {bin_path.name}, ack={ack!r}, extra={extra!r}")

    return input_len, input_len // BYTES_PER_POINT


def run_one(sock: socket.socket, max_result_bytes: int, recv_chunk: int) -> tuple[int, bytes, int]:
    # Trigger accelerator.
    sock.sendall(b"V")

    # New binary result header:
    #   8-byte big-endian voxelize_time_us
    # + 4-byte big-endian result length
    # + 4-byte big-endian response flags
    #
    # Detect old textual error responses early so they do not desynchronize the binary protocol.
    first4 = recv_exact(sock, 4)
    if first4 == b"BUSY":
        raise BoardBusyError("board returned BUSY instead of binary result header")
    if first4 == b"NOIN":
        raise BoardProtocolError("board returned NOINPUT instead of binary result header")
    if first4 == b"BADC":
        raise BoardProtocolError("board returned BADCMD instead of binary result header")

    hdr = first4 + recv_exact(sock, RESP_HDR_BYTES - 4)
    voxelize_time_us, total, resp_flags = struct.unpack("!QII", hdr)

    if total == 0:
        raise BoardProtocolError("board returned zero result length")
    if total > max_result_bytes:
        raise BoardProtocolError(
            f"unreasonable result length from board: {total} bytes > max_result_bytes={max_result_bytes}"
        )

    result = bytearray()
    while len(result) < total:
        want = min(recv_chunk, total - len(result))
        chunk = sock.recv(want)
        if not chunk:
            raise RuntimeError(
                f"socket closed while receiving result: got {len(result)} / {total} bytes"
            )
        result.extend(chunk)

    return voxelize_time_us, bytes(result), resp_flags


def iter_bin_files(input_dir: Path) -> list[Path]:
    files = sorted(input_dir.glob("*.bin"))
    if not files:
        raise FileNotFoundError(f"no .bin files found in {input_dir}")
    return files


def close_socket(s: Optional[socket.socket]) -> None:
    if s is None:
        return
    try:
        s.shutdown(socket.SHUT_RDWR)
    except Exception:
        pass
    try:
        s.close()
    except Exception:
        pass


def main() -> None:
    ap = argparse.ArgumentParser(description="Upload point-cloud .bin files and run voxelize for each frame.")
    ap.add_argument("--host", default=HOST_DEFAULT)
    ap.add_argument("--port", type=int, default=PORT_DEFAULT)
    ap.add_argument("--input-dir", required=True, help="Folder containing KITTI-style .bin point clouds")
    ap.add_argument("--output-dir", required=True, help="Folder to save accelerator outputs")
    ap.add_argument("--save-hex", action="store_true", help="Also save each result as hex txt. This is slow and large.")
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--max-retries", type=int, default=3, help="Retry the same frame after reconnect on TCP/protocol failure")
    ap.add_argument("--retry-sleep", type=float, default=0.5)
    ap.add_argument("--reconnect-every", type=int, default=0,
                    help="Reconnect after N successful frames. 0 keeps one connection. Use 1 for maximum robustness.")
    ap.add_argument("--recv-chunk", type=int, default=256 * 1024,
                    help="Maximum bytes requested per PC-side recv() call")
    ap.add_argument("--socket-rcvbuf", type=int, default=4 * 1024 * 1024)
    ap.add_argument("--socket-sndbuf", type=int, default=4 * 1024 * 1024)
    ap.add_argument("--max-result-bytes", type=int, default=DEFAULT_MAX_RESULT_BYTES)
    ap.add_argument("--allow-board-flags", action="store_true",
                    help="Do not fail/retry when board response flags are non-zero. Default is to fail on DONE_FALLBACK/WAIT_TIMEOUT because the result may be stale or incomplete.")
    args = ap.parse_args()

    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    files = iter_bin_files(input_dir)

    print(f"frames = {len(files)}")
    print(f"connect to {args.host}:{args.port}")
    print(f"response header bytes = {RESP_HDR_BYTES}")

    rows: list[dict[str, object]] = []
    wall_start = time.perf_counter()
    total_hw_us = 0

    s: Optional[socket.socket] = None
    frames_on_current_conn = 0

    try:
        for idx, bin_path in enumerate(files):
            print(f"[{idx + 1}/{len(files)}] upload {bin_path.name}")

            last_exc: Optional[BaseException] = None

            for attempt in range(1, args.max_retries + 2):
                input_bytes: Optional[int] = None
                points: Optional[int] = None
                voxelize_time_us: Optional[int] = None
                result: Optional[bytes] = None
                resp_flags = 0

                try:
                    if (
                        s is None
                        or (args.reconnect_every > 0 and frames_on_current_conn >= args.reconnect_every)
                    ):
                        close_socket(s)
                        s = connect_socket(args.host, args.port, args.timeout, args.socket_rcvbuf, args.socket_sndbuf)
                        frames_on_current_conn = 0

                    frame_wall_start = time.perf_counter()

                    input_bytes, points = upload_one(s, bin_path)
                    voxelize_time_us, result, resp_flags = run_one(s, args.max_result_bytes, args.recv_chunk)

                    if (resp_flags != 0) and (not args.allow_board_flags):
                        append_error_log(
                            output_dir=output_dir,
                            frame_idx=idx,
                            bin_path=bin_path,
                            input_bytes=input_bytes,
                            points=points,
                            voxelize_time_us=voxelize_time_us,
                            result_bytes=len(result),
                            flags=resp_flags,
                            reason="board_returned_nonzero_flags_result_rejected",
                            attempt=attempt,
                        )
                        raise BoardProtocolError(
                            f"board returned non-zero response flags 0x{resp_flags:08X} "
                            f"({decode_resp_flags(resp_flags)}); result rejected"
                        )

                    frame_wall_s = time.perf_counter() - frame_wall_start
                    total_hw_us += voxelize_time_us

                    out_bin = output_dir / f"{bin_path.stem}_pseudo_image.bin"
                    out_bin.write_bytes(result)

                    out_hex = ""
                    if args.save_hex:
                        out_txt = output_dir / f"{bin_path.stem}_pseudo_image.txt"
                        save_as_hex_txt(out_txt, result)
                        out_hex = str(out_txt)

                    flag_desc = decode_resp_flags(resp_flags)

                    # Record board-side abnormal status into error.log.
                    # The most important case is RESP_FLAG_DONE_FALLBACK:
                    #   The board observed idle after busy, but done_latched was not asserted.
                    if resp_flags != 0:
                        if resp_flags & RESP_FLAG_DONE_FALLBACK:
                            reason = "done_latched_missing_idle_after_busy_fallback"
                        elif resp_flags & RESP_FLAG_WAIT_TIMEOUT:
                            reason = "board_wait_done_timeout"
                        else:
                            reason = "board_returned_nonzero_resp_flags"

                        append_error_log(
                            output_dir=output_dir,
                            frame_idx=idx,
                            bin_path=bin_path,
                            input_bytes=input_bytes,
                            points=points,
                            voxelize_time_us=voxelize_time_us,
                            result_bytes=len(result),
                            flags=resp_flags,
                            reason=reason,
                            attempt=attempt,
                        )

                    rows.append({
                        "frame_idx": idx,
                        "input_file": str(bin_path),
                        "input_bytes": input_bytes,
                        "points": points,
                        "voxelize_time_us": voxelize_time_us,
                        "voxelize_time_ms": voxelize_time_us / 1000.0,
                        "result_bytes": len(result),
                        "frame_wall_s": frame_wall_s,
                        "output_bin": str(out_bin),
                        "output_hex": out_hex,
                        "attempt": attempt,
                        "resp_flags": f"0x{resp_flags:08X}",
                        "resp_flag_desc": flag_desc,
                        "done_fallback": int((resp_flags & RESP_FLAG_DONE_FALLBACK) != 0),
                        "wait_timeout": int((resp_flags & RESP_FLAG_WAIT_TIMEOUT) != 0),
                    })

                    frames_on_current_conn += 1
                    print(
                        f"    points={points}, input={input_bytes} bytes, "
                        f"hw={voxelize_time_us / 1000.0:.3f} ms, "
                        f"wall={frame_wall_s:.3f} s, result={len(result)} bytes, "
                        f"flags=0x{resp_flags:08X}({flag_desc}), attempt={attempt}"
                    )
                    break

                except (ConnectionResetError, TimeoutError, socket.timeout, OSError,
                        RuntimeError, BoardProtocolError, BoardBusyError) as e:
                    last_exc = e
                    print(f"    attempt {attempt} failed: {type(e).__name__}: {e}")

                    append_error_log(
                        output_dir=output_dir,
                        frame_idx=idx,
                        bin_path=bin_path,
                        input_bytes=input_bytes,
                        points=points,
                        voxelize_time_us=voxelize_time_us,
                        result_bytes=len(result) if result is not None else None,
                        flags=resp_flags,
                        reason="pc_tcp_or_protocol_failure",
                        attempt=attempt,
                        extra=f"{type(e).__name__}: {e}",
                    )

                    close_socket(s)
                    s = None
                    frames_on_current_conn = 0

                    if attempt <= args.max_retries:
                        time.sleep(args.retry_sleep)
                    else:
                        raise RuntimeError(f"frame {idx} {bin_path.name} failed after retries") from last_exc
    finally:
        close_socket(s)

    wall_total_s = time.perf_counter() - wall_start
    csv_path = output_dir / "summary.csv"

    if rows:
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)

    hw_total_s = total_hw_us / 1_000_000.0

    print("==== summary ====")
    print(f"frames          = {len(rows)} / {len(files)}")
    print(f"sum hw time     = {hw_total_s:.6f} s")
    print(f"hw-only FPS     = {len(rows) / hw_total_s:.3f}" if hw_total_s > 0 else "hw-only FPS     = inf")
    print(f"wall time       = {wall_total_s:.6f} s")
    print(f"end-to-end FPS  = {len(rows) / wall_total_s:.3f}" if wall_total_s > 0 else "end-to-end FPS  = inf")
    print(f"summary saved   = {csv_path}")
    print(f"error log       = {output_dir / 'error.log'}")


if __name__ == "__main__":
    main()


"""

Example:

python E:\verilog\board\lwIP\lwIP\lwIP.py ^
  --input-dir E:\mmdetection3d\data\kitti\training\velodyne_resorted ^
  --output-dir E:\verilog\board\lwIP\lwIP\outputs ^
  --reconnect-every 50 ^
  --max-retries 3 ^
  --timeout 60


  python E:\verilog\board\lwIP\lwIP\lwIP.py ^
  --input-dir E:\mmdetection3d\data\kitti\training\velodyne_test ^
  --output-dir E:\verilog\board\lwIP\lwIP\outputs ^
  --reconnect-every 50 ^
  --max-retries 3 ^
  --timeout 60


"""


