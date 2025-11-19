# vosk_ros2

ROS2 пакет для распознавания речи на основе Vosk.

## Описание

Пакет предоставляет ROS2 узел для непрерывного распознавания речи с использованием библиотеки Vosk. Пакет адаптирован из [whisper_ros](https://github.com/mgonzs13/whisper_ros) для работы с Vosk вместо Whisper.

## Используемые проекты

### Vosk
- **Репозиторий**: [alphacep/vosk](https://github.com/alphacep/vosk)
- **Описание**: Offline open source speech recognition toolkit
- **Лицензия**: Apache 2.0
- **Документация**: https://alphacephei.com/vosk/

Vosk предоставляет Python API для распознавания речи без необходимости подключения к интернету. Пакет использует модель `vosk-model-small-ru` для распознавания русской речи.

### whisper_ros
- **Репозиторий**: [mgonzs13/whisper_ros](https://github.com/mgonzs13/whisper_ros)
- **Описание**: Speech-to-Text based on SileroVAD + whisper.cpp (GGML Whisper) for ROS 2
- **Лицензия**: MIT
- **Автор**: Miguel Ángel González Santamarta

Пакет `vosk_ros2` создан на основе структуры и архитектуры `whisper_ros`, но адаптирован для работы с Vosk вместо Whisper. Сохранена совместимость интерфейсов (сообщения и действия) для обеспечения совместимости с существующим кодом.

## Основные отличия от whisper_ros

- Использует Vosk вместо Whisper (не требует GPU)
- Работает без VAD (Voice Activity Detection) - непрерывное распознавание
- Использует `sounddevice` напрямую вместо `audio_common`
- Python реализация вместо C++
- Модель необходимо скачать отдельно (см. раздел "Установка")

## Зависимости

- Python пакеты:
  - `vosk` - библиотека распознавания речи
  - `sounddevice` - работа с аудио устройствами
  - `rclpy` - ROS2 Python клиент

- ROS2 пакеты:
  - `std_msgs` - стандартные сообщения ROS2
  - `rosidl_default_generators` - генерация интерфейсов

## Установка

```bash
# Перейдите в директорию src вашего рабочего пространства ROS2
cd ~/ros2_ws/src

# Клонируйте этот репозиторий
git clone <repository_url>

# Перейдите в корневую директорию рабочего пространства
cd ~/ros2_ws

# Установите зависимости (если еще не установлены)
rosdep install --from-paths src --ignore-src -r -y

# Установите Python зависимости
pip3 install vosk sounddevice

# Скачайте модель Vosk (см. раздел ниже)
cd src/vosk_ros2
mkdir -p models
cd models
wget https://alphacephei.com/vosk/models/vosk-model-small-ru-0.22.zip
unzip vosk-model-small-ru-0.22.zip
mv vosk-model-small-ru-0.22 vosk-model-small-ru
rm vosk-model-small-ru-0.22.zip

# Вернитесь в корневую директорию рабочего пространства
cd ~/ros2_ws

# Соберите пакет
colcon build --packages-select vosk_ros2

# Загрузите окружение ROS2
source install/setup.bash
```

### Скачивание модели Vosk

Модель Vosk не включена в репозиторий из-за большого размера. Необходимо скачать её вручную:

1. **Выберите модель** на [официальном сайте Vosk](https://alphacephei.com/vosk/models):
   - `vosk-model-small-ru-0.22` - русская модель малого размера (~40 МБ, рекомендуется)
   - `vosk-model-ru-0.22` - русская модель среднего размера (~1.5 ГБ)
   - Другие языки и размеры доступны на сайте

2. **Скачайте и распакуйте модель** в директорию `models/` пакета:
   ```bash
   cd ros_packages/src/vosk_ros2
   mkdir -p models
   cd models
   wget https://alphacephei.com/vosk/models/vosk-model-small-ru-0.22.zip
   unzip vosk-model-small-ru-0.22.zip
   mv vosk-model-small-ru-0.22 vosk-model-small-ru
   rm vosk-model-small-ru-0.22.zip
   ```

3. **Или укажите путь к существующей модели** через параметр `model_path` при запуске:
   ```bash
   ros2 launch vosk_ros2 vosk.launch.py model_path:=/path/to/your/vosk-model
   ```

## Использование

### Запуск узла

```bash
ros2 launch vosk_ros2 vosk.launch.py
```

### Параметры запуска

- `model_path` - путь к модели Vosk (по умолчанию используется модель из пакета)
- `sample_rate` - частота дискретизации (по умолчанию 48000 Hz)
- `device_index` - индекс аудио устройства как строка (по умолчанию "2,0", формат: "0" для простого индекса или "host_index,device_index" для составного)
- `device_name` - имя аудио устройства для поиска (опционально, альтернатива device_index)
- `channels` - количество каналов (по умолчанию 1 - моно)
- `transcription_topic` - имя топика для публикации результатов распознавания (по умолчанию "/audio/transcription")

### Пример с параметрами

```bash
ros2 launch vosk_ros2 vosk.launch.py \
    sample_rate:=16000 \
    device_index:="0" \
    channels:=1 \
    transcription_topic:="/my_custom_topic"
```

**Важно**: Параметр `device_index` должен быть строкой в кавычках:
- `device_index:="0"` - для простого индекса устройства
- `device_index:="2,0"` - для формата "host_index,device_index"

## Топики

- `/audio/transcription` (`vosk_ros2/msg/Transcription`) - публикует результаты распознавания речи (по умолчанию, можно изменить через параметр `transcription_topic`)

## Действия (Actions)

- `/vosk/listen` (`vosk_ros2/action/STT`) - действие для запроса распознавания речи

## Сообщения

### Transcription
- `text` (string) - распознанный текст
- `audio_time` (float32) - время аудио
- `transcription_time` (float32) - время распознавания

## Лицензия

MIT License

## Авторы

- Zhuravliov Pavel (zhuravliov.pav@yandex.ru)
