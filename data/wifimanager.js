
function getWifiScanJson() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function () {
        if (this.readyState == 4 && this.status == 200) {
            var obj = JSON.parse(this.responseText);
            if (obj.scan_result.length) {
                var htmlSrc = '<ul>';
                for (var i = 0; i < obj.scan_result.length; i++) {
                    htmlSrc += '<li><strong>' + obj.scan_result[i].SSID + '</strong> ' + obj.scan_result[i].RSSI + '%</li>';
                }
                htmlSrc += '</ul>';
                document.getElementById("wifilist").innerHTML = htmlSrc;
            }
            console.log(obj);
        }
    };
    xhttp.open("GET", "wifiScan.json", true);
    xhttp.send();
}


function getSecretJson() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function () {
        if (this.readyState == 4 && this.status == 200) {
            //document.getElementById("LEDState").innerHTML =
            var obj = JSON.parse(this.responseText);
            for (var i = 1; i < 4; i++) {
                document.getElementById("ssid" + i).value = obj['ssid' + i];
                document.getElementById("pass" + i).value = obj['pass' + i];
            }
            console.log(obj);
        }
    };
    xhttp.open("GET", "secrets.json", true);
    xhttp.send();
}


function showPassword(id) {
    var x = document.getElementById(id);
    if (x.type === "password") {
        x.type = "text";
    } else {
        x.type = "password";
    }
}

