async function on_Button_10_pressed() {
    try {
      const response = await fetch("http://ESP32stat.local/button10pressed", {
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
  