FROM python:3.11-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY main.py motor_link.py ./
COPY templates ./templates

EXPOSE 8000
EXPOSE 8001
EXPOSE 8002/udp

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]