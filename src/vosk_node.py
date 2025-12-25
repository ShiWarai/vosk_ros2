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

        # Объявляем параметры
        self.declare_parameter('model_path', "")
        self.declare_parameter('audio_topic', '/audio/input')
        self.declare_parameter('transcription_topic', '/audio/transcription')
        self.declare_parameter('max_audio_time', 30.0)  # Максимальное время накопления аудио в секундах
        self.declare_parameter('silence_timeout', 2.0)  # Таймаут тишины для публикации результата (для файлов)
        self.declare_parameter('default_sample_rate', 44100)  # Частота дискретизации по умолчанию для ранней инициализации
        self.declare_parameter('default_channels', 1)  # Количество каналов по умолчанию

        # Получаем параметры
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        audio_topic = self.get_parameter('audio_topic').get_parameter_value().string_value
        
        # Автоматически формируем топик транскрипции из топика аудио
        # По умолчанию: /название_топика_аудио/transcription
        # Можно переопределить через параметр transcription_topic
        transcription_topic_param = self.get_parameter('transcription_topic').get_parameter_value().string_value
        
        # Если transcription_topic равен дефолтному значению, формируем из audio_topic
        # Иначе используем явно указанный топик (для обратной совместимости)
        if transcription_topic_param == '/audio/transcription':
            # Формируем из audio_topic: /название_топика/transcription
            transcription_topic = audio_topic.rstrip('/') + '/transcription'
        else:
            # Используем явно указанный топик (переопределение)
            transcription_topic = transcription_topic_param
        
        self.max_audio_time = self.get_parameter('max_audio_time').get_parameter_value().double_value
        self.silence_timeout = self.get_parameter('silence_timeout').get_parameter_value().double_value
        
        # Параметры аудио по умолчанию для ранней инициализации
        default_sample_rate = self.get_parameter('default_sample_rate').get_parameter_value().integer_value
        default_channels = self.get_parameter('default_channels').get_parameter_value().integer_value
        
        # Параметры аудио будут получены из первого сообщения (если не заданы по умолчанию)
        self.sample_rate = default_sample_rate if default_sample_rate > 0 else None
        self.channels = default_channels if default_channels > 0 else None

        # Загружаем модель Vosk
        try:
            self.model = vosk.Model(model_path)
            # Инициализируем recognizer сразу, если заданы параметры по умолчанию
            if self.sample_rate is not None and self.sample_rate > 0:
                self.rec = vosk.KaldiRecognizer(self.model, self.sample_rate)
                self.get_logger().info(f'Модель Vosk загружена из: {model_path}')
                self.get_logger().info(f'Recognizer инициализирован заранее: частота {self.sample_rate} Hz, каналы {self.channels}')
            else:
                self.rec = None  # Будет создан при первом сообщении
                self.get_logger().info(f'Модель Vosk загружена из: {model_path}')
                self.get_logger().info('Recognizer будет инициализирован при первом аудио сообщении')
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
            
            # Инициализируем recognizer при первом сообщении (если еще не инициализирован)
            if self.rec is None:
                self.sample_rate = audio_info.rate
                self.channels = audio_info.channels
                self.rec = vosk.KaldiRecognizer(self.model, self.sample_rate)
                self.get_logger().info(f'Инициализирован recognizer: частота {self.sample_rate} Hz, каналы {self.channels}')
            elif audio_info.rate != self.sample_rate or audio_info.channels != self.channels:
                # Если формат изменился, пересоздаем recognizer
                self.get_logger().warn(
                    f'Несовпадение формата: ожидается {self.sample_rate} Hz/{self.channels} каналов, '
                    f'получено {audio_info.rate} Hz/{audio_info.channels} каналов. Пересоздаем recognizer.'
                )
                self.sample_rate = audio_info.rate
                self.channels = audio_info.channels
                self.rec = vosk.KaldiRecognizer(self.model, self.sample_rate)
                self.get_logger().info(f'Recognizer пересоздан: частота {self.sample_rate} Hz, каналы {self.channels}')
            
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
            self.get_logger().debug(f'Получено аудио: {len(audio_bytes)} байт, частота: {audio_info.rate} Hz, каналы: {audio_info.channels}')
            
        except Exception as e:
            self.get_logger().error(f'Ошибка обработки аудио сообщения: {e}')

    def _recognition_thread(self):
        """Поток для обработки аудио и распознавания речи."""
        audio_start_time = None
        last_data_time = None  # Время последнего получения данных
        audio_bytes_accumulated = 0
        
        while self.running:
            try:
                data = self.audio_queue.get(timeout=0.5)  # Уменьшили timeout для более частой проверки тишины
                
                # Отслеживаем время начала аудио для этой сессии распознавания
                if audio_start_time is None:
                    audio_start_time = time.time()
                
                # Обновляем время последнего получения данных
                last_data_time = time.time()
                
                # Проверяем таймаут накопления аудио
                elapsed_time = time.time() - audio_start_time
                if elapsed_time > self.max_audio_time:
                    self.get_logger().warn(
                        f'Превышен таймаут накопления аудио ({self.max_audio_time} сек). '
                        f'Сбрасываем счетчик. Накоплено: {audio_bytes_accumulated} байт'
                    )
                    audio_start_time = None
                    last_data_time = None
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
                        last_data_time = None
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
                        last_data_time = None
                        audio_bytes_accumulated = 0
                else:
                    # Частичный результат (логируем только периодически, чтобы не спамить)
                    if audio_bytes_accumulated % (self.sample_rate * self.channels * self.bytes_per_sample) < len(data):
                        # Логируем примерно раз в секунду
                        partial = json.loads(self.rec.PartialResult())
                        partial_text = partial.get('partial', '')
                        if partial_text:
                            self.get_logger().debug(f'Частичное распознавание: "{partial_text}" (накоплено: {audio_bytes_accumulated} байт)')
            except queue.Empty:
                # Проверяем таймаут при отсутствии новых данных
                if audio_start_time is not None and last_data_time is not None:
                    time_since_last_data = time.time() - last_data_time
                    elapsed_time = time.time() - audio_start_time
                    
                    # Если прошло достаточно времени без данных (silence_timeout), публикуем результат
                    # Это работает для файлов - когда файл закончился, данные перестают приходить
                    if time_since_last_data >= self.silence_timeout and audio_bytes_accumulated > 0:
                        # Публикуем финальный результат после тишины
                        if self.rec is not None:
                            final_result = json.loads(self.rec.FinalResult())
                            final_text = final_result.get('text', '')
                            if final_text:
                                audio_duration = audio_bytes_accumulated / (self.sample_rate * self.channels * self.bytes_per_sample)
                                audio_duration = min(audio_duration, self.max_audio_time)
                                self._publish_transcription(final_text, audio_duration, 0.0)
                                self.get_logger().info(f'Опубликован финальный результат после тишины ({self.silence_timeout} сек): "{final_text}"')
                            else:
                                self.get_logger().debug(f'Тишина {self.silence_timeout} сек: накоплено {audio_bytes_accumulated} байт, но текст не распознан')
                        
                        # Сбрасываем recognizer для следующей сессии
                        if self.rec is not None:
                            self.rec = vosk.KaldiRecognizer(self.model, self.sample_rate)
                        audio_start_time = None
                        last_data_time = None
                        audio_bytes_accumulated = 0
                    elif elapsed_time > self.max_audio_time:
                        # Публикуем финальный результат перед сбросом (долгий таймаут)
                        if self.rec is not None and audio_bytes_accumulated > 0:
                            # Получаем финальный результат
                            final_result = json.loads(self.rec.FinalResult())
                            final_text = final_result.get('text', '')
                            if final_text:
                                audio_duration = audio_bytes_accumulated / (self.sample_rate * self.channels * self.bytes_per_sample)
                                audio_duration = min(audio_duration, self.max_audio_time)
                                self._publish_transcription(final_text, audio_duration, 0.0)
                                self.get_logger().info(f'Опубликован финальный результат после таймаута: "{final_text}"')
                            else:
                                self.get_logger().debug(f'Таймаут накопления аудио: накоплено {audio_bytes_accumulated} байт, но текст не распознан')
                        else:
                            self.get_logger().warn(
                                f'Таймаут накопления аудио при отсутствии данных ({self.max_audio_time} сек). '
                                f'Сбрасываем счетчик. Накоплено: {audio_bytes_accumulated} байт'
                            )
                        # Сбрасываем recognizer для следующей сессии
                        if self.rec is not None:
                            self.rec = vosk.KaldiRecognizer(self.model, self.sample_rate)
                        audio_start_time = None
                        last_data_time = None
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

