#!/usr/bin/env python3
"""
ROS2 узел для распознавания речи на основе Vosk.

Узел непрерывно слушает аудио вход и публикует результаты распознавания.
Основан на voice_control.py, но адаптирован для ROS2.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory

import sounddevice as sd
import queue
import vosk
import json
import time
import threading
import os

from vosk_ros2.msg import Transcription
from vosk_ros2.action import STT


class VoskNode(Node):
    """ROS2 узел для распознавания речи на основе Vosk."""

    def __init__(self):
        super().__init__('vosk_node')

        # Получаем путь к модели по умолчанию из share директории пакета
        try:
            pkg_share_dir = get_package_share_directory('vosk_ros2')
            default_model_path = os.path.join(pkg_share_dir, 'models', 'vosk-model-small-ru')
        except:
            default_model_path = '/home/orangepi/models/vosk-model-small-ru'

        # Объявляем параметры
        self.declare_parameter('model_path', default_model_path)
        self.declare_parameter('sample_rate', 48000)
        self.declare_parameter('device_index', '2,0')
        self.declare_parameter('device_name', '')
        self.declare_parameter('channels', 1)
        self.declare_parameter('transcription_topic', '/audio/transcription')

        # Получаем параметры
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        device_index_param = self.get_parameter('device_index').get_parameter_value().string_value
        device_name = self.get_parameter('device_name').get_parameter_value().string_value
        self.channels = self.get_parameter('channels').get_parameter_value().integer_value
        transcription_topic = self.get_parameter('transcription_topic').get_parameter_value().string_value

        # Парсим device_index
        self.device_index = self._parse_device_index(device_index_param, device_name)

        # Загружаем модель Vosk
        try:
            self.model = vosk.Model(model_path)
            self.rec = vosk.KaldiRecognizer(self.model, self.sample_rate)
            self.get_logger().info(f'Модель Vosk загружена из: {model_path}')
        except Exception as e:
            self.get_logger().fatal(f'Ошибка загрузки модели Vosk: {e}')
            raise

        # Очередь для аудио данных
        self.audio_queue = queue.Queue()
        
        # Отслеживание обработки аудио
        self.bytes_per_sample = 2  # int16 = 2 байта
        self.last_recognition_time = None

        # Публикаторы
        self.transcription_pub = self.create_publisher(
            Transcription,
            transcription_topic,
            10
        )

        # Action сервер
        self._action_server = ActionServer(
            self,
            STT,
            'listen',
            self.execute_callback
        )

        # Запускаем поток захвата аудио
        self.running = True
        self.audio_thread = threading.Thread(target=self._audio_capture_thread, daemon=True)
        self.audio_thread.start()

        # Запускаем поток распознавания
        self.recognition_thread = threading.Thread(target=self._recognition_thread, daemon=True)
        self.recognition_thread.start()

        self.get_logger().info(f'Узел Vosk запущен (частота: {self.sample_rate} Гц, устройство: {self.device_index}, топик: {transcription_topic})')

    def _parse_device_index(self, device_index_param, device_name):
        """Парсит параметр device_index для получения индекса устройства для sounddevice."""
        if device_name:
            # Ищем устройство по имени
            devices = sd.query_devices()
            for i, device in enumerate(devices):
                if device_name.lower() in device['name'].lower():
                    return i
            self.get_logger().warn(f'Устройство "{device_name}" не найдено')
        
        # Пытаемся распарсить device_index как строку "host_index,device_index"
        if ',' in device_index_param:
            try:
                parts = device_index_param.split(',')
                host_index = int(parts[0])
                device_index = int(parts[1])
                # sounddevice использует кортеж для host,device
                return (host_index, device_index)
            except ValueError:
                pass
        
        # Пытаемся распарсить как целое число
        try:
            return int(device_index_param)
        except ValueError:
            return None

    def _audio_callback(self, indata, frames, time_info, status):
        """Callback для потока аудио входа."""
        if status:
            self.get_logger().warn(f'Статус аудио callback: {status}')
        self.audio_queue.put(bytes(indata))

    def _audio_capture_thread(self):
        """Поток для захвата аудио."""
        try:
            with sd.RawInputStream(
                samplerate=self.sample_rate,
                dtype='int16',
                channels=self.channels,
                callback=self._audio_callback,
                device=self.device_index
            ):
                self.get_logger().info('Захват аудио запущен')
                while self.running:
                    time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f'Ошибка захвата аудио: {e}')
            self.running = False

    def _recognition_thread(self):
        """Поток для обработки аудио и распознавания речи."""
        audio_start_time = None
        audio_bytes_accumulated = 0
        
        while self.running:
            try:
                data = self.audio_queue.get(timeout=1.0)
                
                # Отслеживаем время начала аудио для этой сессии распознавания
                if audio_start_time is None:
                    audio_start_time = time.time()
                
                # Накопляем байты аудио
                audio_bytes_accumulated += len(data)
                
                # Измеряем время обработки распознавания
                transcription_start = time.time()
                
                if self.rec.AcceptWaveform(data):
                    transcription_time = time.time() - transcription_start
                    result = json.loads(self.rec.Result())
                    text = result.get('text', '')
                    if text:
                        # Вычисляем длительность аудио из накопленных байтов
                        audio_duration = audio_bytes_accumulated / (self.sample_rate * self.channels * self.bytes_per_sample)
                        self._publish_transcription(text, audio_duration, transcription_time)
                        
                        # Сбрасываем для следующего распознавания
                        audio_start_time = None
                        audio_bytes_accumulated = 0
                else:
                    # Частичный результат
                    partial = json.loads(self.rec.PartialResult())
                    partial_text = partial.get('partial', '')
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Ошибка распознавания: {e}')

    def _publish_transcription(self, text, audio_time=0.0, transcription_time=0.0):
        """Публикует результат распознавания."""
        msg = Transcription()
        msg.text = text
        msg.audio_time = audio_time
        msg.transcription_time = transcription_time
        
        self.transcription_pub.publish(msg)
        self.get_logger().info(f'Распознавание: {text}')

    def execute_callback(self, goal_handle):
        """Callback action сервера для действия STT."""
        # Примечание: Vosk не поддерживает prompt или grammar config напрямую,
        # но мы сохраняем совместимость с интерфейсом whisper_ros
        
        # Для непрерывного распознавания просто подтверждаем goal
        # и возвращаем последнее распознавание когда доступно
        result_msg = STT.Result()
        
        # Ждем распознавание (с таймаутом)
        start_time = time.time()
        timeout = 10.0  # Таймаут 10 секунд
        
        transcription_received = False
        latest_transcription = None
        
        while not transcription_received and (time.time() - start_time) < timeout:
            # Проверяем, был ли goal отменен
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return STT.Result()
            
            # Пытаемся получить распознавание из очереди или ждем
            time.sleep(0.1)
            
            # В реальной реализации можно сохранять последнее распознавание
            # Пока просто подтверждаем и возвращаем пустой результат
        
        # Завершаем goal успешно
        goal_handle.succeed()
        result_msg.transcription = Transcription()
        result_msg.transcription.text = latest_transcription if latest_transcription else ""
        
        return result_msg

    def destroy_node(self):
        """Очистка при уничтожении узла."""
        self.running = False
        
        # Ждем завершения потоков
        if hasattr(self, 'audio_thread') and self.audio_thread.is_alive():
            self.audio_thread.join(timeout=2.0)
        if hasattr(self, 'recognition_thread') and self.recognition_thread.is_alive():
            self.recognition_thread.join(timeout=2.0)
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = VoskNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f'Ошибка в узле: {e}')
    finally:
        if node:
            try:
                node.destroy_node()
            except:
                pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()

