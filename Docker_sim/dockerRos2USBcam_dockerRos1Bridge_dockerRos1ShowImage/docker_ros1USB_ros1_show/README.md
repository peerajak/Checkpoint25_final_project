docker build -t test:ros1_usbcam_show  .
docker run --rm -it test:ros1_usbcam_show bash
docker compose up
