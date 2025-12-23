# MIT License
#
# Copyright (c) 2024
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Генерирует описание запуска для узла Vosk ROS2."""

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
        default_value='/audio/input',
        description='ROS2 топик для подписки на аудио данные'
    )

    transcription_topic_arg = DeclareLaunchArgument(
        'transcription_topic',
        default_value='/audio/transcription',
        description='Имя топика для публикации результатов распознавания'
    )

    max_audio_time_arg = DeclareLaunchArgument(
        'max_audio_time',
        default_value='30.0',
        description='Максимальное время накопления аудио в секундах перед принудительным сбросом (защита от переполнения)'
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
        }],
        output='screen'
    )

    return LaunchDescription([
        model_path_arg,
        audio_topic_arg,
        transcription_topic_arg,
        max_audio_time_arg,
        vosk_node,
    ])
