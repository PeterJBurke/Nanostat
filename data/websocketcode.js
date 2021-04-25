// Javascript code to set up a websocket.

var m_url_JS = "ws://nanostat.local:81/";
//var m_url_JS = "ws://192.168.1.44:81/";
// var url = "ws://192.168.4.1:1337/";

var m_websocket;
var m_canvas_JS;
var context;
var dataPlot;
var maxDataPoints = 20; // max points in browser cache

// This is called when the page finishes loading
function init() {

    // Assign page elements to variables
    m_canvas_JS = document.getElementById("m_canvas");

    // create chart:

    // dataPlot = new Chart(document.getElementById("m_canvas"), {
    //     type: 'line',
    //     data: {
    //         labels: [],
    //         datasets: [{
    //             data: [],
    //             label: "Temperature (C)",
    //             borderColor: "#3e95cd",
    //             fill: false
    //         }]
    //     },
    //     options: {
    //         scales: {
    //             y: {
    //                 beginAtZero: true
    //             }
    //         }
    //     }
    // });

    // Connect to WebSocket server
    wsConnect(m_url_JS);
}

// Call this to connect to the WebSocket server
function wsConnect(m_url_JS) {

    // Connect to WebSocket server
    m_url_JS = "ws://" + window.location.hostname + ":81/"
    // console.log(m_url_JS);
    m_websocket = new WebSocket(m_url_JS);

    // Assign callbacks
    m_websocket.onopen = function (evt) { onOpen(evt) };
    m_websocket.onclose = function (evt) { onClose(evt) };
    m_websocket.onmessage = function (evt) { onMessage(evt) };
    m_websocket.onerror = function (evt) { onError(evt) };

}

// Called when a WebSocket connection is established with the server
function onOpen(evt) {

    // Log connection state
    console.log("Connected");

    // Enable button
    // button.disabled = false;

    // Get the current state of the LED
    // doSend("getLEDState");
}

// Called when the WebSocket connection is closed
function onClose(evt) {

    // Log disconnection state
    console.log("Disconnected");

    // Disable button
    // button.disabled = true;

    // Try to reconnect after a few seconds
    setTimeout(function () { wsConnect(m_url_JS) }, 2000);
}

// remove excess data from plot
function removeData() {
    dataPlot.data.labels.shift();
    dataPlot.data.datasets[0].data.shift();
}

// add data to plot (through chart object push method...)
function addData(label, data) {
    if (dataPlot.data.labels.length > maxDataPoints) removeData();
    dataPlot.data.labels.push(label);
    dataPlot.data.datasets[0].data.push(data);
    dataPlot.update();
}

// Called when a message is received from the server
function onMessage(evt) {

    // Print out our received message
    console.log("Received: " + evt.data);

    // Add data to chart.js plot:
    // var data = JSON.parse(evt.data); // data is a json object; data.time assumes format it {"value",123456}
    // var today = new Date();
    // var t = today.getHours() + ":" + today.getMinutes() + ":" + today.getSeconds();
    // addData(t, data.value); // add new data points to plot one by one...
    // console.log("Parsed version: " + data.value);
    // console.log(t);
    // console.log(data);

    var m_json_obj = JSON.parse(evt.data);
    console.log(m_json_obj);
    var m_voltage_array = m_json_obj.Voltage;
    var m_current_array = m_json_obj.Current;
    console.log(m_voltage_array);
    console.log(m_current_array);

    TESTER = document.getElementById('plotly-tester');
    // Plotly.newPlot(TESTER, [{
    //     x: [1, 2, 3, 4, 5],
    //     y: [1, 2, 4, 8, 16]
    // }], {
    //     margin: { t: 0 }
    // });
    
    var trace_Voltammogram = {
        x: m_voltage_array,
        y: m_current_array,
        mode: 'markers',
        type: 'scatter'
      };
      
    //   var trace2 = {
    //     x: [2, 3, 4, 5],
    //     y: [16, 5, 11, 9],
    //     mode: 'lines',
    //     type: 'scatter'
    //   };
      
      
      var data_Voltammogram = [trace_Voltammogram];
      
      Plotly.newPlot('plotly-tester', data_Voltammogram);
    

}

function addData(label, data) {
    if (dataPlot.data.labels.length > maxDataPoints) removeData();
    dataPlot.data.labels.push(label);
    dataPlot.data.datasets[0].data.push(data);
    dataPlot.update();
}

// Called when a WebSocket error occurs
function onError(evt) {
    console.log("ERROR: " + evt.data);
}

// Sends a message to the server (and prints it to the console)
function doSend(message) {
    console.log("Sending: " + message);
    websocket.send(message);
}

// Slider calls this to set data rate:
function sendDataRate() {
    var dataRate = document.getElementById("dataRateSlider").value;
    m_websocket.send(dataRate);
    dataRate = 1.0 * dataRate;
    document.getElementById("dataRateLabel").innerHTML = "Rate: " + dataRate.toFixed(2) + "Hz";
}

// Call the init function as soon as the page loads
window.addEventListener("load", init, false);


