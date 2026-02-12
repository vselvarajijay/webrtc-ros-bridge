# Pure Python app-server (no ROS2)
FROM python:3.11-slim

WORKDIR /app

COPY main.py .
RUN pip install --no-cache-dir fastapi "uvicorn[standard]" websockets

EXPOSE 8001
CMD ["python", "main.py"]
