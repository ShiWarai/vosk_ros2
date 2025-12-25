#!/usr/bin/env python3
"""
Launch файл для запуска Vosk с настройками для Telegram аудио.

Использование:
ros2 launch vosk_ros2 vosk_telegram.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Генерирует описание запуска для узла Vosk с настройками для Telegram."""

    # Получаем путь к модели по умолчанию из директории share пакета
    from ament_index_python.packages import get_package_share_directory
    import os
    pkg_share_dir = get_package_share_directory('vosk_ros2')
    default_model_path = os.path.join(pkg_share_dir, 'models', 'vosk-model-small-ru-0.22')

    # Аргументы запуска
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=default_model_path,
        description='Путь к директории модели Vosk'
    )

    audio_topic_arg = DeclareLaunchArgument(
        'audio_topic',
        default_value='/telegram/audio',
        description='ROS2 топик для подписки на аудио данные из Telegram'
    )

    transcription_topic_arg = DeclareLaunchArgument(
        'transcription_topic',
        default_value='/telegram/audio/transcription',
        description='Имя топика для публикации результатов распознавания (по умолчанию формируется из audio_topic)'
    )

    max_audio_time_arg = DeclareLaunchArgument(
        'max_audio_time',
        default_value='30.0',
        description='Максимальное время накопления аудио в секундах перед принудительным сбросом (защита от переполнения)'
    )

    silence_timeout_arg = DeclareLaunchArgument(
        'silence_timeout',
        default_value='2.0',
        description='Таймаут тишины в секундах для публикации результата после окончания потока данных (для файлов)'
    )

    default_sample_rate_arg = DeclareLaunchArgument(
        'default_sample_rate',
        default_value='16000',
        description='Частота дискретизации для Telegram аудио (16kHz)'
    )

    default_channels_arg = DeclareLaunchArgument(
        'default_channels',
        default_value='1',
        description='Количество каналов для Telegram аудио (моно)'
    )

    # Узел
    vosk_node = Node(
        package='vosk_ros2',
        executable='vosk_node',
        name='vosk_node',
        namespace='vosk',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'audio_topic': LaunchConfiguration('audio_topic'),
            'transcription_topic': LaunchConfiguration('transcription_topic'),
            'max_audio_time': LaunchConfiguration('max_audio_time'),
            'silence_timeout': LaunchConfiguration('silence_timeout'),
            'default_sample_rate': LaunchConfiguration('default_sample_rate'),
            'default_channels': LaunchConfiguration('default_channels'),
        }],
        output='screen'
    )

    return LaunchDescription([
        model_path_arg,
        audio_topic_arg,
        transcription_topic_arg,
        max_audio_time_arg,
        silence_timeout_arg,
        default_sample_rate_arg,
        default_channels_arg,
        vosk_node,
    ])


