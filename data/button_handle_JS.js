{/* <button onclick= "window.location.href = '/wifi_manager.html'"> Wifi Setup </button> */ }

let temporary_boolean = true;
async function on_Button_1_pressed() {
  console.log(window.location.hostname)
  try {
    // const response = await fetch("http://nanostat.local/button1pressed", {
    const response = await fetch("/button1pressed", {
      method: "PUT",
      body: JSON.stringify({ on: temporary_boolean }),
      headers: {
        "Content-Type": "application/json",
      },
    });
  } catch (error) {
    alert("Request failed - check the console");
    console.error(error);
  }
}
async function on_Button_2_pressed() {
  try {
    const response = await fetch("/button2pressed", {
      method: "PUT",
      body: JSON.stringify({ on: temporary_boolean }),
      headers: {
        "Content-Type": "application/json",
      },
    });
  } catch (error) {
    alert("Request failed - check the console");
    console.error(error);
  }
}
async function on_Button_3_pressed() {
  try {
    const response = await fetch("/button3pressed", {
      method: "PUT",
      body: JSON.stringify({ on: temporary_boolean }),
      headers: {
        "Content-Type": "application/json",
      },
    });
  } catch (error) {
    alert("Request failed - check the console");
    console.error(error);
  }
}
async function on_Button_4_pressed() {
  try {
    const response = await fetch("/button4pressed", {
      method: "PUT",
      body: JSON.stringify({ on: temporary_boolean }),
      headers: {
        "Content-Type": "application/json",
      },
    });
  } catch (error) {
    alert("Request failed - check the console");
    console.error(error);
  }
}
async function on_Button_5_pressed() {
  try {
    const response = await fetch("/button5pressed", {
      method: "PUT",
      body: JSON.stringify({ on: temporary_boolean }),
      headers: {
        "Content-Type": "application/json",
      },
    });
  } catch (error) {
    alert("Request failed - check the console");
    console.error(error);
  }
}
async function on_Button_6_pressed() {
  try {
    const response = await fetch("/button6pressed", {
      method: "PUT",
      body: JSON.stringify({ on: temporary_boolean }),
      headers: {
        "Content-Type": "application/json",
      },
    });
  } catch (error) {
    alert("Request failed - check the console");
    console.error(error);
  }
}
async function on_Button_7_pressed() {
  try {
    const response = await fetch("/button7pressed", {
      method: "PUT",
      body: JSON.stringify({ on: temporary_boolean }),
      headers: {
        "Content-Type": "application/json",
      },
    });
  } catch (error) {
    alert("Request failed - check the console");
    console.error(error);
  }
}
async function on_Button_8_pressed() {
  try {
    const response = await fetch("/button8pressed", {
      method: "PUT",
      body: JSON.stringify({ on: temporary_boolean }),
      headers: {
        "Content-Type": "application/json",
      },
    });
  } catch (error) {
    alert("Request failed - check the console");
    console.error(error);
  }
}
async function on_Button_9_pressed() {
  try {
    const response = await fetch("/button9pressed", {
      method: "PUT",
      body: JSON.stringify({ on: temporary_boolean }),
      headers: {
        "Content-Type": "application/json",
      },
    });
  } catch (error) {
    alert("Request failed - check the console");
    console.error(error);
  }
}
async function on_Button_10_pressed() {
  try {
    const response = await fetch("/button10pressed", {
      method: "PUT",
      body: JSON.stringify({ on: temporary_boolean }),
      headers: {
        "Content-Type": "application/json",
      },
    });
  } catch (error) {
    alert("Request failed - check the console");
    console.error(error);
  }
}


