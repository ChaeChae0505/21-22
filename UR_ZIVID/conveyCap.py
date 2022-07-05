import datetime
from zivid import Application, Settings
from sample_utils.paths import get_sample_data_path
from sample_utils.settings_from_file import get_settings_from_yaml
import zivid
import time
from pathlib import Path

class capture():
    def zdf():
        app = Application()
        camera = app.connect_camera()

        settings = Settings()
        settings.acquisitions.append(Settings.Acquisition())
        settings.acquisitions[0].aperture = 3.0 #5.6
        settings.acquisitions[0].exposure_time = datetime.timedelta(microseconds=8333)
        settings.processing.filters.outlier.removal.enabled = True
        settings.processing.filters.outlier.removal.threshold = 5.0

        with camera.capture(settings) as frame:
            frame.save("3.zdf")


    """
    This example shows how to capture 2D images from the Zivid camera.
    """


    def CamPng(result): # camera app before not on
        app = zivid.Application()

        print("Connecting to camera")
        camera = app.connect_camera()

        print("Configuring 2D settings")
        settings_2d = zivid.Settings2D()
        settings_2d.acquisitions.append(zivid.Settings2D.Acquisition())
        settings_2d.acquisitions[0].exposure_time = datetime.timedelta(microseconds=100000)
        settings_2d.acquisitions[0].aperture = 2.33#5.0
        settings_2d.acquisitions[0].gain = 1.0
        settings_2d.acquisitions[0].brightness = 0.0
        settings_2d.processing.color.balance.red = 1.0
        settings_2d.processing.color.balance.green = 1.0
        settings_2d.processing.color.balance.blue = 1.0
        settings_2d.processing.color.gamma = 1.0
        print("Capturing 2D frame")
        with camera.capture(settings_2d) as frame_2d:
            print("Getting RGBA image")
            image = frame_2d.image_rgba()
            rgba = image.copy_data()
            # pixel 값 알려주는 거임 걍
            pixel_row = 1
            pixel_col = 1
            pixel = rgba[pixel_row, pixel_col]
            print(f"Color at pixel ({pixel_row},{pixel_col}): R:{pixel[0]} G:{pixel[1]} B:{pixel[2]} A:{pixel[3]}")

            image_file = result
            print(f"Saving image to file: {image_file}")
            image.save(image_file)

    def png(camera, result):  # camera app on after

        print("Connecting to camera")


        print("Configuring 2D settings")
        settings_2d = zivid.Settings2D()
        settings_2d.acquisitions.append(zivid.Settings2D.Acquisition())
        settings_2d.acquisitions[0].exposure_time = datetime.timedelta(microseconds=100000)
        settings_2d.acquisitions[0].aperture = 2.33 #2.33 30 cm
        settings_2d.acquisitions[0].brightness = 0.0
        settings_2d.acquisitions[0].gain = 1.00
        settings_2d.processing.color.balance.red = 1.0
        settings_2d.processing.color.balance.green = 1.0
        settings_2d.processing.color.balance.blue = 1.0
        settings_2d.processing.color.gamma = 1.0

        with camera.capture(settings_2d) as frame_2d:
            print("Getting RGBA image")
            image = frame_2d.image_rgba()
            rgba = image.copy_data()
            # pixel 값 알려주는 거임 걍
            pixel_row = 1
            pixel_col = 1
            pixel = rgba[pixel_row, pixel_col]
            print(f"Color at pixel ({pixel_row},{pixel_col}): R:{pixel[0]} G:{pixel[1]} B:{pixel[2]} A:{pixel[3]}")

            image_file = result
            print(f"Saving image to file: {image_file}")
            image.save(image_file)
    def final(camera, result):  # camera app on after

        print("Connecting to camera")


        print("Configuring 2D settings")
        settings_2d = zivid.Settings2D()
        settings_2d.acquisitions.append(zivid.Settings2D.Acquisition())
        settings_2d.acquisitions[0].exposure_time = datetime.timedelta(microseconds=100000)
        settings_2d.acquisitions[0].aperture = 5.0 #2.33 30 cm
        settings_2d.acquisitions[0].brightness = 0.0
        settings_2d.acquisitions[0].gain = 1.00
        settings_2d.processing.color.balance.red = 1.0
        settings_2d.processing.color.balance.green = 1.0
        settings_2d.processing.color.balance.blue = 1.0
        settings_2d.processing.color.gamma = 1.0

        with camera.capture(settings_2d) as frame_2d:
            print("Getting RGBA image")
            image = frame_2d.image_rgba()
            rgba = image.copy_data()
            # pixel 값 알려주는 거임 걍
            pixel_row = 1
            pixel_col = 1
            pixel = rgba[pixel_row, pixel_col]
            print(f"Color at pixel ({pixel_row},{pixel_col}): R:{pixel[0]} G:{pixel[1]} B:{pixel[2]} A:{pixel[3]}")

            image_file = result
            print(f"Saving image to file: {image_file}")
            image.save(image_file)

    def hdr():
        app = Application()
        camera = app.connect_camera()

        settings = Settings(
            acquisitions=[
                Settings.Acquisition(aperture=aperture) for aperture in (10.90, 5.80, 2.83)
            ]
        )
        with camera.capture(settings) as hdr_frame:
            hdr_frame.save("result.zdf")


if __name__ == "__main__":
    for i in range(1, 100):
        PATH = 'Yolo/'
        IMG_DIR = PATH +  str(i) + '.png'
        capture.CamPng(IMG_DIR)
        time.sleep(3)
        print("Save:", IMG_DIR)
