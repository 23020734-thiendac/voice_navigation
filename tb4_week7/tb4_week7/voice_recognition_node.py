"""Offline voice-to-text bridge with Vosk and a text-topic test input."""

from __future__ import annotations

import json
from pathlib import Path
import queue
import time
import unicodedata
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String


class VoiceRecognitionNode(Node):
    """Publish completed speech phrases on /voice_command.

    Vosk is optional at import time so the navigation stack can still be
    tested with `ros2 topic pub /voice_command_text ...` on a clean machine.
    """

    def __init__(self) -> None:
        super().__init__('voice_recognition_node')
        self.declare_parameter('backend', 'auto')
        self.declare_parameter('model_path', '')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('device', -1)
        self.declare_parameter('publish_partial', False)
        # Keep endpoint results briefly so a long spoken route is published as
        # one command instead of being cut into several short commands.
        self.declare_parameter('command_hold_sec', 2.5)

        self.output_pub = self.create_publisher(String, '/voice_command', 10)
        self.audio_queue: queue.Queue[bytes] = queue.Queue()
        self.recognizer = None
        self.audio_stream = None
        self.audio_backend_ready = False
        self.pending_text = []
        self.pending_deadline = 0.0
        self.timer = self.create_timer(0.05, self.process_audio)
        self.flush_timer = self.create_timer(0.05, self.flush_pending)
        self.start_backend()

    @staticmethod
    def _normalize(text: str) -> str:
        text = text.lower().replace('đ', 'd')
        text = unicodedata.normalize('NFD', text)
        return ''.join(
            char for char in text
            if unicodedata.category(char) != 'Mn')

    def canonicalize_command(self, text: str) -> str:
        """Return a stable Vietnamese phrase for terminal and navigation."""
        normalized = self._normalize(text)
        if any(token in normalized for token in ('dung', 'dung lai', 'dung robot')):
            return 'dừng'

        area_patterns: List[Tuple[str, str]] = [
            ('phong ngu', 'phòng ngủ'),
            ('phong khach', 'phòng khách'),
            ('phong bep', 'phòng bếp'),
            ('khu vuc tap the hinh', 'chỗ tập'),
            ('cho tap', 'chỗ tập'),
        ]
        mentions: List[Tuple[int, str]] = []
        for pattern, label in area_patterns:
            start = 0
            while True:
                index = normalized.find(pattern, start)
                if index < 0:
                    break
                mentions.append((index, label))
                start = index + len(pattern)
        mentions.sort(key=lambda item: item[0])
        route: List[str] = []
        for _, label in mentions:
            if not route or route[-1] != label:
                route.append(label)
        if len(route) == 1:
            return f'đi tới {route[0]}'
        if len(route) == 2:
            return f'đi từ {route[0]} tới {route[1]}'
        if len(route) > 2:
            return (
                f'đi từ {route[0]} qua '
                + ' qua '.join(route[1:-1])
                + ' rồi tới ' + route[-1])
        return text

    def publish_text(self, text: str, source: str) -> None:
        text = self.canonicalize_command(text)
        output = String()
        output.data = text
        self.output_pub.publish(output)
        self.get_logger().info(f'Lenh giong noi ({source}): {text}')

    def start_backend(self) -> None:
        backend = str(self.get_parameter('backend').value).lower()
        model_path = str(self.get_parameter('model_path').value).strip()
        if backend == 'text':
            self.get_logger().info(
                'Che do text-topic: nhan cau lenh tu /voice_command_text.')
            return
        if backend not in ('auto', 'vosk'):
            self.get_logger().warning(
                f'Backend {backend} chua duoc ho tro; dung text-topic.')
            return
        if not model_path or not Path(model_path).is_dir():
            self.get_logger().warning(
                'Chua co Vosk model. Dung /voice_command_text de test; '
                'muon nhan microphone hay dat model_path:=/duong/dan/model.')
            return
        try:
            import sounddevice as sd
            from vosk import KaldiRecognizer, Model
            # Restrict decoding to the commands used by this exercise.
            # This greatly reduces Vietnamese small-model hallucinations from
            # background speech/noise (for example, random unrelated phrases).
            # Chỉ nhận câu lệnh tiếng Việt. Không đưa các từ English như
            # room/living/kitchen vào grammar vì mô hình nhỏ dễ trả nhầm chúng.
            command_grammar = [
                'đi đến phòng ngủ', 'đi tới phòng ngủ', 'phòng ngủ',
                'đi đến phòng khách', 'đi tới phòng khách', 'phòng khách',
                'đi đến phòng bếp', 'đi tới phòng bếp', 'phòng bếp',
                'đi đến khu vực tập thể hình', 'khu vực tập thể hình',
                'đi đến chỗ tập', 'chỗ tập',
                'dừng', 'dung', 'dừng lại', 'dung lai',
                'dừng robot', 'dung robot',
                'đi từ phòng ngủ qua phòng khách rồi tới phòng bếp',
                'đi từ phòng ngủ đến phòng khách rồi đến phòng bếp'
            ]
            self.recognizer = KaldiRecognizer(
                Model(model_path),
                float(self.get_parameter('sample_rate').value),
                json.dumps(command_grammar, ensure_ascii=False))
            device = int(self.get_parameter('device').value)

            def callback(indata, frames, time_info, status):
                if status:
                    self.get_logger().warning(str(status))
                self.audio_queue.put(bytes(indata))

            kwargs = {
                'samplerate': int(self.get_parameter('sample_rate').value),
                'blocksize': 4000,
                'dtype': 'int16',
                'channels': 1,
                'callback': callback,
            }
            if device >= 0:
                kwargs['device'] = device
            self.audio_stream = sd.RawInputStream(**kwargs)
            self.audio_stream.start()
            self.audio_backend_ready = True
            self.get_logger().info('Vosk microphone da san sang.')
        except Exception as exc:  # optional dependency/device errors are recoverable
            self.get_logger().warning(
                f'Khong khoi dong duoc Vosk microphone: {exc}. '
                'Van nhan /voice_command_text de kiem thu.')

    def process_audio(self) -> None:
        if not self.audio_backend_ready or self.recognizer is None:
            return
        try:
            audio = self.audio_queue.get_nowait()
        except queue.Empty:
            return
        try:
            if self.recognizer.AcceptWaveform(audio):
                result = json.loads(self.recognizer.Result())
                text = str(result.get('text', '')).strip()
                if text:
                    # Vosk emits a final result at each short silence. Delay
                    # publication slightly and merge adjacent results, which
                    # preserves long commands such as a multi-room route.
                    self.pending_text.append(text)
                    self.pending_deadline = (
                        time.monotonic() +
                        float(self.get_parameter('command_hold_sec').value))
            elif bool(self.get_parameter('publish_partial').value):
                result = json.loads(self.recognizer.PartialResult())
                text = str(result.get('partial', '')).strip()
                if text:
                    self.get_logger().debug(f'Vosk partial: {text}')
        except Exception as exc:
            self.get_logger().warning(f'Loi xu ly audio Vosk: {exc}')

    def flush_pending(self) -> None:
        if not self.pending_text or time.monotonic() < self.pending_deadline:
            return
        text = ' '.join(self.pending_text).strip()
        self.pending_text.clear()
        self.pending_deadline = 0.0
        if text:
            self.publish_text(text, source='vosk')

    def destroy_node(self):
        if self.audio_stream is not None:
            try:
                self.audio_stream.stop()
                self.audio_stream.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoiceRecognitionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
