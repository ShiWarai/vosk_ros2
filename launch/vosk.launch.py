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
    """Генерирует описание launch для узла Vosk ROS2."""

    # Получаем путь к модели по умолчанию из share директории пакета
    from ament_index_python.packages import get_package_share_directory
    import os
    try:
        pkg_share_dir = get_package_share_directory('vosk_ros2')
        default_model_path = os.path.join(pkg_share_dir, 'models', 'vosk-model-small-ru')
    except:
        default_model_path = '/home/orangepi/models/vosk-model-small-ru'

    # Аргументы launch
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=default_model_path,
        description='Путь к директории модели Vosk'
    )

    sample_rate_arg = DeclareLaunchArgument(
        'sample_rate',
        default_value='48000',
        description='Частота дискретизации аудио в Гц'
    )

    device_index_arg = DeclareLaunchArgument(
        'device_index',
        default_value='2,0',
        description='Индекс аудио устройства как строка (может быть "0" для простого индекса или "host_index,device_index" для формата "2,0")'
    )

    device_name_arg = DeclareLaunchArgument(
        'device_name',
        default_value='',
        description='Имя аудио устройства для поиска (альтернатива device_index)'
    )

    channels_arg = DeclareLaunchArgument(
        'channels',
        default_value='1',
        description='Количество аудио каналов (1 для моно, 2 для стерео)'
    )

    transcription_topic_arg = DeclareLaunchArgument(
        'transcription_topic',
        default_value='/audio/transcription',
        description='Имя топика для публикации результатов распознавания'
    )

    # Узел
    vosk_node = Node(
        package='vosk_ros2',
        executable='vosk_node',
        name='vosk_node',
        namespace='vosk',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'sample_rate': LaunchConfiguration('sample_rate'),
            'device_index': LaunchConfiguration('device_index'),
            'device_name': LaunchConfiguration('device_name'),
            'channels': LaunchConfiguration('channels'),
            'transcription_topic': LaunchConfiguration('transcription_topic'),
        }],
        output='screen'
    )

    return LaunchDescription([
        model_path_arg,
        sample_rate_arg,
        device_index_arg,
        device_name_arg,
        channels_arg,
        transcription_topic_arg,
        vosk_node,
    ])

