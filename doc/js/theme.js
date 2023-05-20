// To reload mermaid graphs
document.getElementById('light').onclick = function () {
    location.reload();
};

document.getElementById('rust').onclick = function () {
    location.reload();
};

document.getElementById('coal').onclick = function () {
    location.reload();
};

document.getElementById('navy').onclick = function () {
    location.reload();
};

document.getElementById('ayu').onclick = function () {
    location.reload();
};

if (window.location.href.indexOf('http://localhost/') == 0) {
    document.getElementById("logo-dc").src = "/images/dc.png"
    document.getElementById("logo-dc").href = "/"
} else if (window.location.href.indexOf('https://minipada.github.io/ros2_data_collection/') == 0) {
    let level = window.location.href.split("https://minipada.github.io/ros2_data_collection/")[1].split("/").length - 1

    document.getElementById("logo-dc").src = "../".repeat(level) + "images/dc.png"
    document.getElementById("logo-dc").href = "https://minipada.github.io/ros2_data_collection"
}
