# Captura y Procesamiento de Imágenes con ESP32-CAM 🧑‍💻

Este proyecto utiliza un ESP32-CAM para capturar imágenes, aplicar filtros de procesamiento de imágenes como Sobel y Thresholding, y guardar las imágenes procesadas en una tarjeta SD.

![l (1)](https://github.com/user-attachments/assets/11139cc0-08aa-4d26-af0e-21eb2bba36e8)

## Librerías Utilizadas

- `esp_camera.h`: Configuración y manejo de la cámara.
- `FS.h` y `SD_MMC.h`: Para el manejo de la tarjeta SD.
- `TJpg_Decoder.h`: Decodificación de imágenes JPEG.
- `EEPROM.h`: Lectura y escritura en memoria flash.

## Funcionalidades Principales

### Captura de Imágenes

El proyecto captura imágenes en formato JPEG utilizando la cámara configurada para tamaño VGA.

### Procesamiento de Imágenes

1. **Conversión RGB565 a RGB888**: Conversión de formato para aplicar filtros.
2. **Filtro Sobel**: Detección de bordes en escala de grises.
3. **Thresholding**: Conversión a imagen binaria según un umbral.
4. **Detección de Contornos**: Identificación de regiones de interés.
5. **Dibujo de Cuadro Delimitador**: Visualización de los contornos detectados en la imagen original.

### Almacenamiento

Las imágenes procesadas se guardan en la tarjeta SD en formato BMP junto con su cabecera correspondiente.


