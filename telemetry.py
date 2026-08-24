from communication import SerialComm
import threading

class RF():
    def __init__(self, rf_comm):
        self.ser = SerialComm()
        self.ser.connect_to(rf_comm)
        self.rf_data = None
        self._lock = threading.Lock()   # Protects rf_data string from read/write race

    def send_data(self, data):
        """Send AUV telemetry to GCS over RF UART.
        UART TX and RX use separate hardware buffers, so sending while the
        recieve_data thread is reading is safe - no lock needed here."""
        self.ser.send_data(data)

    def recieve_data(self):
        """
        Blocking receive - reads one full line from the RF UART port.
        Called from a dedicated daemon thread in a loop.
        Stores the result thread-safely in self.rf_data.
        """
        received = self.ser.recieve_data()  # Blocks until a full line arrives
        if received:
            with self._lock:
                self.rf_data = received.strip()

    def get_rf_data(self):
        """Thread-safe getter for the latest received RF string."""
        with self._lock:
            return self.rf_data