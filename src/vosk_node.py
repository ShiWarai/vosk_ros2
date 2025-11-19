#!/usr/bin/env python3
"""
ROS2 узел для распознавания речи на основе Vosk.

Узел подписывается на топик аудио и публикует результаты распознавания.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory

import queue
import vosk
import json
import time
import threading
import os
import numpy as np

from vosk_ros2.msg import Transcription
from vosk_ros2.action import STT
from audio_common_msgs.msg import AudioStamped


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
        self.declare_parameter('audio_topic', '/audio/input')
        self.declare_parameter('transcription_topic', '/audio/transcription')
        self.declare_parameter('max_audio_time', 30.0)  # Максимальное время накопления аудио в секундах

        # Получаем параметры
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        audio_topic = self.get_parameter('audio_topic').get_parameter_value().string_value
        transcription_topic = self.get_parameter('transcription_topic').get_parameter_value().string_value
        self.max_audio_time = self.get_parameter('max_audio_time').get_parameter_value().double_value
        
        # Параметры аудио будут получены из первого сообщения
        self.sample_rate = None
        self.channels = None

        # Загружаем модель Vosk (частота будет установлена при первом сообщении)
        try:
            self.model = vosk.Model(model_path)
            self.rec = None  # Будет создан при первом сообщении
            self.get_logger().info(f'Модель Vosk загружена из: {model_path}')
        except Exception as e:
            self.get_logger().fatal(f'Ошибка загрузки модели Vosk: {e}')
            raise

        # Очередь для аудио данных
        self.audio_queue = queue.Queue()
        
        # Отслеживание обработки аудио
        self.bytes_per_sample = 2  # int16 = 2 байта

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

        # Подписываемся на топик аудио
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.audio_subscription = self.create_subscription(
            AudioStamped,
            audio_topic,
            self.audio_callback,
            sensor_qos
        )

        # Запускаем поток распознавания
        self.running = True
        self.recognition_thread = threading.Thread(target=self._recognition_thread, daemon=True)
        self.recognition_thread.start()

        self.get_logger().info(f'Узел Vosk запущен (топик аудио: {audio_topic}, топик транскрипции: {transcription_topic})')

    def _extract_audio_data(self, audio_data_msg):
        """Извлечь аудио данные из сообщения и конвертировать в int16 байты для Vosk."""
        # Извлекаем данные в numpy array
        data = None
        if hasattr(audio_data_msg, 'float32_data') and len(audio_data_msg.float32_data) > 0:
            data = np.array(audio_data_msg.float32_data, dtype=np.float32)
        elif hasattr(audio_data_msg, 'int16_data') and len(audio_data_msg.int16_data) > 0:
            # Уже int16, можно использовать напрямую
            return np.array(audio_data_msg.int16_data, dtype=np.int16).tobytes()
        elif hasattr(audio_data_msg, 'int32_data') and len(audio_data_msg.int32_data) > 0:
            data = np.array(audio_data_msg.int32_data, dtype=np.int32).astype(np.float32) / 2147483648.0
        elif hasattr(audio_data_msg, 'int8_data') and len(audio_data_msg.int8_data) > 0:
            data = np.array(audio_data_msg.int8_data, dtype=np.int8).astype(np.float32) / 128.0
        elif hasattr(audio_data_msg, 'uint8_data') and len(audio_data_msg.uint8_data) > 0:
            data = (np.array(audio_data_msg.uint8_data, dtype=np.uint8).astype(np.float32) - 128) / 128.0
        
        if data is None or len(data) == 0:
            return None
        
        # Конвертируем float32 [-1, 1] в int16 байты
        data = np.clip(data, -1.0, 1.0)
        audio_int16 = (data * 32767).astype(np.int16)
        return audio_int16.tobytes()

    def audio_callback(self, msg):
        """Callback для обработки аудио сообщений из топика."""
        try:
            audio_info = msg.audio.info
            
            # Инициализируем recognizer при первом сообщении
            if self.rec is None:
                self.sample_rate = audio_info.rate
                self.channels = audio_info.channels
                self.rec = vosk.KaldiRecognizer(self.model, self.sample_rate)
                self.get_logger().info(f'Инициализирован recognizer: частота {self.sample_rate} Hz, каналы {self.channels}')
            
            # Проверяем, что частота совпадает
            if audio_info.rate != self.sample_rate:
                self.get_logger().warn(f'Несовпадение частоты: ожидается {self.sample_rate} Hz, получено {audio_info.rate} Hz')
                return
            
            # Извлекаем и конвертируем аудио данные
            audio_bytes = self._extract_audio_data(msg.audio.audio_data)
            if audio_bytes is None:
                return
            
            # Если многоканальное, берем только первый канал
            if self.channels > 1:
                # Конвертируем обратно в numpy для обработки каналов
                audio_array = np.frombuffer(audio_bytes, dtype=np.int16)
                audio_array = audio_array.reshape(-1, self.channels)[:, 0]  # Берем первый канал
                audio_bytes = audio_array.tobytes()
            
            # Добавляем в очередь для обработки
            self.audio_queue.put(audio_bytes)
            
        except Exception as e:
            self.get_logger().error(f'Ошибка обработки аудио сообщения: {e}')

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
                
                # Проверяем таймаут накопления аудио
                elapsed_time = time.time() - audio_start_time
                if elapsed_time > self.max_audio_time:
                    self.get_logger().warn(
                        f'Превышен таймаут накопления аудио ({self.max_audio_time} сек). '
                        f'Сбрасываем счетчик. Накоплено: {audio_bytes_accumulated} байт'
                    )
                    audio_start_time = None
                    audio_bytes_accumulated = 0
                    continue
                
                # Накопляем байты аудио
                audio_bytes_accumulated += len(data)
                
                # Проверяем переполнение (максимум ~2GB для int в Python, но лучше ограничить)
                if self.sample_rate is not None and self.channels is not None:
                    max_bytes = int(self.max_audio_time * self.sample_rate * self.channels * self.bytes_per_sample)
                    if audio_bytes_accumulated > max_bytes:
                        self.get_logger().warn(
                            f'Превышен лимит накопленных байт ({max_bytes}). '
                            f'Сбрасываем счетчик. Текущее значение: {audio_bytes_accumulated} байт'
                        )
                        audio_start_time = None
                        audio_bytes_accumulated = 0
                        continue
                
                # Измеряем время обработки распознавания
                transcription_start = time.time()
                
                if self.rec.AcceptWaveform(data):
                    transcription_time = time.time() - transcription_start
                    result = json.loads(self.rec.Result())
                    text = result.get('text', '')
                    if text:
                        # Вычисляем длительность аудио из накопленных байтов
                        audio_duration = audio_bytes_accumulated / (self.sample_rate * self.channels * self.bytes_per_sample)
                        # Ограничиваем максимальное значение для безопасности
                        audio_duration = min(audio_duration, self.max_audio_time)
                        self._publish_transcription(text, audio_duration, transcription_time)
                        
                        # Сбрасываем для следующего распознавания
                        audio_start_time = None
                        audio_bytes_accumulated = 0
                else:
                    # Частичный результат
                    partial = json.loads(self.rec.PartialResult())
                    partial_text = partial.get('partial', '')
            except queue.Empty:
                # Проверяем таймаут при отсутствии новых данных
                if audio_start_time is not None:
                    elapsed_time = time.time() - audio_start_time
                    if elapsed_time > self.max_audio_time:
                        self.get_logger().warn(
                            f'Таймаут накопления аудио при отсутствии данных ({self.max_audio_time} сек). '
                            f'Сбрасываем счетчик.'
                        )
                        audio_start_time = None
                        audio_bytes_accumulated = 0
                continue
            except Exception as e:
                self.get_logger().error(f'Ошибка распознавания: {e}')
                # Сбрасываем при ошибке
                audio_start_time = None
                audio_bytes_accumulated = 0

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
        self.get_logger().info('Завершение работы узла Vosk...')
        self.running = False
        
        # Ждем завершения потока распознавания
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

