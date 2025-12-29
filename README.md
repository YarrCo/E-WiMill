# E-WiMill

ESP32-S3 + SD (SDMMC 1-bit) + USB MSC. Проект рассчитан на безопасную передачу файлов между ПК/станком и ESP без повреждения FAT32. В основе - жесткое разделение режимов доступа к SD.

## Статус

- Этап 1 (MVP-02): стабильный USB Mass Storage + локальные CLI операции с SD.
- Этап 2 (MVP-03A): Setup Mode с AP, Web UI, сохранением настроек и переходом в STA + mDNS.

## Аппаратная конфигурация

- MCU: ESP32-S3
- SD (SDMMC 1-bit, GPIO matrix):
  - CLK: GPIO12
  - CMD: GPIO11
  - D0: GPIO13
  - D3/CS: GPIO10 (удерживается в HIGH)
  - Частота: 40 MHz default, fallback 20 MHz
  - ФС: FAT32
- USB: native USB ESP32-S3 (MSC)
- Кнопка Setup: GPIO21 (active-low, на GND, внутренняя подтяжка)
- RGB LED: GPIO48 (WS2812/NeoPixel, 1 светодиод)

## Ключевой принцип (критично)

SD-карта не используется одновременно:
- как USB-накопитель
- и как локальная файловая система ESP

Любая попытка совмещать доступ приводит к RAW/зависаниям/повреждению данных. Поэтому переключение только вручную.

## Этап 1 (MVP-02): USB MSC + CLI/VFS

### Что происходит при старте

1) Инициализируется SD в RAW режиме (SDMMC 1-bit).
2) Запускается TinyUSB MSC.
3) Устройство сразу в режиме `USB_ATTACHED`.

### Режимы

- `USB_ATTACHED` - SD отдана ПК как флешка (FAT32). Локальные файловые операции ESP запрещены.
- `USB_DETACHED` - USB MSC отключен, SD примонтирована как `/sdcard` для ESP.

### Переключение режимов (CLI)

Через UART (115200):

- `usb status` - текущий режим
- `usb attach` - отдать SD наружу как флешку
- `usb detach` - отключить MSC, смонтировать `/sdcard`

### Файловые команды (только в USB_DETACHED)

- `ls [path]`
- `info`
- `mkdir <dir>`
- `rm <file>`
- `cat <file>` (первые 256 байт hex+ascii)
- `touch <file> <n>` (создать файл из n нулей, выполняется в фоне)
- `sdtest [mb] [kHz] [buf N]` (тест записи, выполняется в фоне)
- `sd freq [kHz]` (20000..40000)

Если USB_ATTACHED - команды возвращают BUSY.

### Последовательность работы (пример)

1) Включить питание -> устройство сразу как USB флешка.
2) На ПК копируем файлы.
3) Для локальной работы ESP: `usb detach`.
4) Команды `ls/info/touch/...` работают по `/sdcard`.
5) Вернуть флешку наружу: `usb attach`.

## Этап 2 (MVP-03A): Setup Mode + Wi-Fi

### Вход в Setup Mode

- Удерживать кнопку GPIO21 >= 5 сек при старте, или
- Удерживать кнопку GPIO21 >= 5 сек в normal режиме.

### Поведение в Setup Mode

- Поднимается AP `E-WiMill-XXXX` (XXXX = последние 2 байта MAC).
- Пароль: `wimill1234`.
- IP: `192.168.4.1`.
- Web UI: `http://192.168.4.1:<port>` (по умолчанию `8080`).
- RGB LED: отдельный узнаваемый режим (пульсация). В normal режиме - медленное дыхание цветом (6,87,33).

### UI и API

- `GET /` - страница Setup.
- `GET /api/status` - JSON статуса (mode, ssid, sta_ip, last_sta_ip, rssi, web_port и т.д.).
- `POST /api/config` - сохраняет настройки в NVS.

### Переход AP -> STA (Apply)

Поток:

1) Нажать Apply -> конфиг сохраняется.
2) STA подключение стартует, AP остается активным (AP+STA).
3) Если подключение успешно:
   - Поднимается mDNS: `http://<dev_name>.local:<port>`
   - AP выключается
4) Если ошибка (таймаут 30 сек): остаемся в AP, в статусе появляется `sta_error`.

### mDNS имя

- dev_name нормализуется: нижний регистр, пробелы/`_` -> `-`, только `a-z0-9-`.
- Если имя пустое - fallback `ewimill-XXXX`.

### Где хранятся настройки

NVS (namespace `wimill`). Настройки переживают перепрошивку. Для сброса - `idf.py erase-flash`.

## Этап 3 (MVP-03B): Web File Manager

### Что добавлено

- Web файловый менеджер для `/sdcard` (только в `USB_DETACHED`).
- API:
  - `GET /api/fs/list?path=/`
  - `POST /api/fs/upload` (multipart, fallback)
  - `POST /api/fs/upload_raw?path=/&name=FILE` (быстрый путь)
  - `GET /api/fs/download?path=/file`
  - `POST /api/fs/mkdir`, `POST /api/fs/delete`, `POST /api/fs/rename`

### Проблема и решение по скорости upload

**Проблема:** скорость загрузки по Wi-Fi падала до 40-60 KB/s на больших файлах.  
Причина: маленькие `recv` буферы + запись на SD в том же обработчике (много мелких `fwrite`, блокировки и рост задержек).

**Решение, которое помогло:**

1) Увеличен `recv` буфер до 32KB.  
2) Внедрен pipeline: HTTP handler быстро читает данные и кладет их в ring-buffer (512KB, PSRAM), отдельная writer task пишет на SD крупными блоками.  
3) Добавлен быстрый endpoint `upload_raw` (octet-stream) без multipart-обработки. UI по умолчанию использует raw и падает на multipart только при ошибке.

**Результат:** стабильные ~600-700 KB/s на 20-60 MB файлах (без провала скорости).

### Путь передачи файлов

**Wi-Fi -> SD (Web File Manager):**
1) `usb detach` (SD примонтирована как `/sdcard`).
2) В Web UI вкладка Storage, upload файла.
3) Запись идет в `/sdcard` через `.part` + `rename`.
4) `usb attach` — SD отдается ПК как флешка.

**SD -> ПК (USB MSC):**
1) `usb attach` (SD уходит в USB Mass Storage).
2) ПК/станок видит FAT32-диск, копирует файлы.
3) `usb detach` — SD снова доступна ESP как `/sdcard`.

### Пример быстрого upload (raw)

PowerShell (Windows):

```powershell
$ip = "192.168.1.45"
$port = 8080
$file = "C:\temp\test.bin"
Invoke-WebRequest -Method Post -InFile $file "http://$ip`:$port/api/fs/upload_raw?path=/&name=$(Split-Path $file -Leaf)&overwrite=1"
```

## Сборка и прошивка

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

## Примечания

- Скорость MSC ограничена малыми USB-пакетами и синхронными SD-операциями.
- SDMMC 1-bit работает на 40 MHz (fallback 20 MHz при ошибках инициализации).
- В проект добавлен `components/mdns`, чтобы mDNS работал в IDF 5.5.1.
