
function getWifiScanJson() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function () {
        if (this.readyState == 4 && this.status == 200) {
            var obj = JSON.parse(this.responseText);
            if (obj.scan_result.length) {
                var htmlSrc = '<ul class="list-group">';
                for (var i = 0; i < obj.scan_result.length; i++) {
                    htmlSrc += '<li class="list-group-item"><strong>' + obj.scan_result[i].SSID + '</strong> ' + obj.scan_result[i].RSSI + ' dBm</li>';
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


$('form').submit(function (e) {
  e.preventDefault();
  var form = $('#upload_form')[0];
  var data = new FormData(form);
  $.ajax({
    url: '/edit',
    type: 'POST',
    data: data,
    contentType: false,
    processData: false,
    xhr: function () {
      var xhr = new window.XMLHttpRequest();
      xhr.upload.addEventListener('progress', function (evt) {
        if (evt.lengthComputable) {
          var per = evt.loaded / evt.total;
          $('#prg').html('progress: ' + Math.round(per * 100) + '%');
        }
      }, false);
      return xhr;
    },
    success: function (d, s) {
      console.log('success!');
      getFileList();
    },
    error: function (a, b, c) {
    }
  });
});



function deleteFile(fileName) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function () {
    if (this.readyState == 4 && this.status == 200) {
      //var obj =  JSON.parse();
      //console.log(this.responseText);
      getFileList();
    }
  };
  if (confirm('Are you sure you want to delete the file ?')) {
    xhttp.open("DELETE", "/edit?file=/" + fileName, true);
    xhttp.send();
  }


}

function getFileList() {
  console.log('starting getFileList!');
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function () {
    if (this.readyState == 4 && this.status == 200) {
      var obj = JSON.parse(this.responseText);

      if (obj.length) {
        // var htmlSrc = '<ul>';
        var htmlSrc = '<ul class="list-group">';
        for (var i = 0; i < obj.length; i++) {
          if (obj[i].type == 'file') {
            // htmlSrc += '<li>' + obj[i].name + ' <a href="#" onclick="deleteFile(\'' + obj[i].name + '\')">X</a></li>';
            htmlSrc += '<li class="list-group-item">' + obj[i].name + ' <a href="#" onclick="deleteFile(\'' + obj[i].name + '\')">X</a></li>';
          }
        }
        htmlSrc += '</ul>';
        document.getElementById("filelist").innerHTML = htmlSrc;
      }
      console.log(obj);
      // console.log(htmlSrc);
    }
  };
  xhttp.open("GET", "/list?dir=/", true);
  xhttp.send();
}
