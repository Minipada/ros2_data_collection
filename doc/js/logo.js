window.addEventListener('load', function () {
    let navbar = document.getElementById('sidebar');
    var logo_dc = document.createElement("img");
    logo_dc.src = "/images/dc.png"
    logo_dc.id = 'logo-dc';
    navbar.prepend(logo_dc);
});
