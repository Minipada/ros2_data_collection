import os
import pathlib

import png
import qrcode
import typer
from jinja2 import Environment, FileSystemLoader
from PIL import Image, ImageDraw, ImageFont
from qrcode.image.styledpil import StyledPilImage

app = typer.Typer(help="Generate QR codes")


def create_qr_img(*, data: str, qr_path: str) -> qrcode.image.styledpil.StyledPilImage:
    qr = qrcode.QRCode(error_correction=qrcode.constants.ERROR_CORRECT_H)
    qr.add_data(data)
    qr_img = qr.make_image(
        image_factory=StyledPilImage,
        # TODO Zbar fails to recognize the qrcode with the image.
        # embeded_image_path=(
        #     pathlib.Path(os.path.dirname(os.path.realpath(__file__)))
        #     / "templates"
        #     / "model_qr"
        #     / "ros_logo_large.png"
        # ).as_posix(),
    )
    qr_img.save(qr_path)
    return qr_img


def create_qr_img_w_text(
    *,
    data: str,
    qr_img: qrcode.image.styledpil.StyledPilImage,
    qr_path: str,
    final_path: str,
):
    height_text = 75
    height_qr = qr_img.pixel_size
    width_qr = qr_img.pixel_size
    height_new_img = height_qr + height_text
    width_new_img = width_qr

    # Write all white in image
    f = open(final_path, "wb")  # binary mode is important
    w = png.Writer(width_new_img, height_new_img, greyscale=False)
    w.write(f, [[255] * width_new_img * 3] * height_new_img)  # *3 for RGB
    f.close()

    max_font_size = 60
    max_position = 25

    # Add title with dynamic size based on qr code data length
    with Image.open(final_path) as img:
        d = ImageDraw.Draw(img)
        text_pix = width_new_img - 50  # 25 left, 25 right
        font_pix_ratio = 40 / 24  # For a font of 40, it needs to take around 24px
        font_size = int(text_pix / len(data) * font_pix_ratio)
        if font_size > max_font_size:
            font_size = max_font_size
        font = ImageFont.truetype("NotoMono-Regular.ttf", font_size)
        qr_img = Image.open(qr_path)
        img.paste(qr_img, (5, 60))
        pos_ratio = (10 - len(data)) / 1.75
        position = 25 * pos_ratio if pos_ratio > 1 else max_position
        d.text((position, 15), data, fill=(0), font=font)
        img.save(final_path)


@app.command()
def create_img(
    data: str = typer.Option(..., help="QR code data"),
    path: str = typer.Option(..., help="Path where to save it"),
    keep_qr: bool = typer.Option(
        default=False, help="Keep image of QR Code without text"
    ),
):
    qr_path = (pathlib.Path(path) / f"qr_{data}.png").as_posix()
    final_path = (pathlib.Path(path) / f"qrcode_{data}.png").as_posix()
    qr_img = create_qr_img(data=data, qr_path=qr_path)
    create_qr_img_w_text(
        data=data, qr_img=qr_img, qr_path=qr_path, final_path=final_path
    )
    if not keep_qr:
        os.unlink(qr_path)
    print(f"Image saved: {final_path}")


@app.command()
def create_model(
    data: str = typer.Option(..., help="QR code data"),
    models_dir: str = typer.Option(..., help="ROS2 Package model directory"),
):
    environment = Environment(
        loader=FileSystemLoader(
            (
                pathlib.Path(os.path.dirname(os.path.realpath(__file__))) / "templates"
            ).as_posix()
        )
    )
    model_config_template = environment.get_template("model_qr/model.config.jinja2")
    model_config_content = model_config_template.render(data=data)

    model_sdf_template = environment.get_template("model_qr/model.sdf.jinja2")
    model_sdf_content = model_sdf_template.render(data=data)

    dae_template = environment.get_template("model_qr/dae.jinja2")
    dae_content = dae_template.render(data=data)

    dir_main = pathlib.Path(models_dir) / f"qrcode_{data}"
    dir_dae = pathlib.Path(models_dir) / f"qrcode_{data}" / "meshes"
    dir_textures = (
        pathlib.Path(models_dir) / f"qrcode_{data}" / "materials" / "textures"
    )
    dir_main.mkdir(exist_ok=True, parents=True)
    dir_textures.mkdir(exist_ok=True, parents=True)
    dir_dae.mkdir(exist_ok=True, parents=True)

    # Save image
    qr_path = (pathlib.Path(dir_textures) / f"qr_{data}.png").as_posix()
    final_path = (pathlib.Path(dir_textures) / f"qrcode_{data}.png").as_posix()
    qr_img = create_qr_img(data=data, qr_path=qr_path)
    create_qr_img_w_text(
        data=data, qr_img=qr_img, qr_path=qr_path, final_path=final_path
    )
    os.unlink(qr_path)

    with open(
        (pathlib.Path(dir_main) / f"model.config").as_posix(),
        mode="w",
        encoding="utf-8",
    ) as message:
        message.write(model_config_content)

    with open(
        (pathlib.Path(dir_main) / f"model.sdf").as_posix(), mode="w", encoding="utf-8"
    ) as message:
        message.write(model_sdf_content)

    with open(
        (pathlib.Path(dir_dae) / f"qrcode_{data}.dae").as_posix(),
        mode="w",
        encoding="utf-8",
    ) as message:
        message.write(dae_content)


if __name__ == "__main__":
    app()
