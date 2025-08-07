

//funktionen til scan fra bufferen
 void ScanQRCodeFromBuffer(void)
{
    const int width = 640;
    const int height = 480;

    quirc_t *qr = quirc_new();
    if (!qr) {
        printf("Could not allocate QR decoder\r\n");
        return;
    }

    if (quirc_resize(qr, width, height) < 0) {
        printf("Failed to resize QR decoder\r\n");
        quirc_destroy(qr);
        return;
    }

    // Step 1: få pointer til framebuffer og konverter til grayscale
    uint8_t *gray = quirc_begin(qr, NULL, NULL);
    uint8_t *src = (uint8_t *)BUFFER_ADDRESS;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int index = (y * width + x) * 2;
            uint16_t pixel = src[index] | (src[index + 1] << 8);

            uint8_t r = (pixel >> 11) & 0x1F;
            uint8_t g = (pixel >> 5) & 0x3F;
            uint8_t b = pixel & 0x1F;

            // Grayscale-konvertering (simple version)
            uint8_t gray_value = (r << 3) * 0.3 + (g << 2) * 0.59 + (b << 3) * 0.11;
            gray[y * width + x] = gray_value;
        }
    }

    quirc_end(qr);

    // Step 2: detect og decode
    int count = quirc_count(qr);
    printf("QR codes found: %d\r\n", count);

    for (int i = 0; i < count; i++) {
        struct quirc_code code;
        struct quirc_data data;

        quirc_extract(qr, i, &code);
        if (quirc_decode(&code, &data) == QUIRC_SUCCESS) {
            printf("QR DECODED: %s\r\n", data.payload);
        } else {
            printf("QR decode failed\r\n");
        }
    }

    quirc_destroy(qr);
}

//i løkken:
if (ISP_BackgroundProcess(&hcamera_isp) != ISP_OK)
{
    BSP_LED_Toggle(LED_RED);
}

ScanQRCodeFromBuffer();

HAL_Delay(1000);  // valgfrit delay
