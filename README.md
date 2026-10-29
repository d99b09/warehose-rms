# Warehouse RMS2

1. **Должно быть установлено заранее:** Ubuntu 24.04, Python 3.12, `apt` и `sudo`. Источники пакетов ROS 2 Jazzy уже подключены.

2. **Установить через APT:** пакеты из [requirements-system.txt](requirements-system.txt). ROS-библиотеки, сообщения, TF, CMake и pytest устанавливаются как зависимости указанных пакетов. Выполните из корня проекта:

   ```bash
   xargs -r -a requirements-system.txt sudo apt install -y
   ```

## Сборка проекта

```bash
cd ros_ws && colcon build --symlink-install
```

## Запуск FastAPI

Один раз установите Python-зависимости из корня проекта:

```zsh
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r requirements-web.txt
```

После сборки ROS-пакетов запустите сервер:

```zsh
./scripts/run-fastapi.sh
```

Веб интерфейс доступен по адресу http://localhost:8000/


Во всех трёх скриптах первый аргумент — `normal` или `stress`; по умолчанию `normal`:

```zsh
./scripts/run-simulation.sh stress
./scripts/run-fastapi.sh stress
./scripts/run-all.sh stress
```

Первые две команды запускают компоненты отдельно, третья — оба вместе с одной картой.
