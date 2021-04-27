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
    var m_time_array = m_json_obj.Time;
    console.log(m_voltage_array);
    console.log(m_current_array);
    console.log(m_time_array);


    var trace_IV = {
        x: m_voltage_array,
        y: m_current_array,
        mode: 'markers',
        type: 'scatter'
    };
    var data_IV = [trace_IV];

    var m_IV_layout = {
        // title: 'IV Curve',
        showlegend: false,
        margin: {
            l: 50,
            r: 5,
            b: 50,
            t: 1,
            pad: 4
        },
        xaxis: {
            title: { text: 'Voltage (mV)' }
        },
        yaxis: {
            title: { text: 'Current (microA)' }
        }
    };


    // Plotly.newPlot('plotly-IV', data_IV, m_IV_layout, {scrollZoom: true}, {editable: true}, {responsive: true});
    Plotly.newPlot('plotly-IV', data_IV, m_IV_layout, { scrollZoom: true, editable: true, responsive: true });


    var trace_IvsTime = {
        x: m_time_array,
        y: m_current_array,
        mode: 'markers',
        type: 'scatter',
        name: "Current"
    };
    var trace_VvsTime = {
        x: m_time_array,
        y: m_voltage_array,
        mode: 'markers',
        yaxis: 'y2',
        type: 'scatter',
        name: "Voltage"
    };

    var data_IVvsTime = [trace_IvsTime, trace_VvsTime];


    var m_2yaxis_layout = {
        margin: {
            l: 50,
            r: 5,
            b: 50,
            t: 1,
            pad: 4
        },
        xaxis: {
            title: { text: 'Time (ms)' }
        },
        yaxis: { title: 'Current (microA)' },
        yaxis2: {
            title: 'Voltage (mV)',
            titlefont: { color: 'rgb(148, 103, 189)' },
            tickfont: { color: 'rgb(148, 103, 189)' },
            overlaying: 'y',
            side: 'right'
        }
    };

    Plotly.newPlot('plotly-IvsTime', data_IVvsTime, m_2yaxis_layout, { scrollZoom: true, editable: true, responsive: true });


    // var trace_IvsTime = {
    //     x: m_time_array,
    //     y: m_current_array,
    //     mode: 'markers',
    //     type: 'scatter'
    // };
    // var data_IvsTime = [trace_IvsTime];
    // Plotly.newPlot('plotly-IvsTime', data_IvsTime);

    // var trace_VvsTime = {
    //     x: m_time_array,
    //     y: m_voltage_array,
    //     mode: 'markers',
    //     type: 'scatter'
    // };
    // var data_VvsTime = [trace_VvsTime];
    // Plotly.newPlot('plotly-VvsTime', data_VvsTime);


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


