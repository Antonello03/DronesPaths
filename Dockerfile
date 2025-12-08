FROM --platform=linux/amd64 python:3.9-slim

WORKDIR /app

# Set Python to unbuffered mode
ENV PYTHONUNBUFFERED=1

# Copy requirements and install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy all project files
COPY . .

# Default command
CMD ["python", "-u", "main.py", "data/buildings/Building2.txt"]

