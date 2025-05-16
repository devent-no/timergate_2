<script>
import BreakItem from "./components/BreakItem.vue";
import Pole from "./components/Pole.vue";
import MainLayout from "./layouts/MainLayout.vue";
import TimerView from "./views/TimerView.vue";


export default {
  components: {
    BreakItem,
    Pole,
    MainLayout,
    TimerView
  },
  data() {
    return {
      count: 0,
      poles: [],
      lookup: new Map(),
      breaks: {},
      passages: [], // Array for passeringer
      settings: {},
      socket: null,
      socket_ready: false,
      hostname: "timergate.local",
      useIpAddress: false,
      ipAddress: "192.168.4.1",
      connectionSettings: {
        visible: false
      },
      interval: null,
      time: null,
      show_advanced: false,
      
      // Status for å veksle mellom grensesnitt
      useNewInterface: false
    };
  },
  computed: {
    // Beregn serveradresse basert på innstilling
    serverAddress() {
      return this.useIpAddress ? this.ipAddress : this.hostname;
    }
  },
  methods: {
    init() {
      this.socket = new WebSocket("ws://" + this.serverAddress + "/ws");
      this.socket.onopen = this.onSocketOpen;
      this.socket.onmessage = this.onSocketMessage;
      this.socket.onerror = this.onSockerError;
    },
    clearTimes() {
      this.breaks = {};
    },
    async syncTime() {
      console.log(`Synkroniserer tid via ${this.serverAddress}...`);
      try {
        const timestamp = Math.floor(Date.now() / 1000);
        
        const response = await fetch(`http://${this.serverAddress}/api/v1/time/sync`, {
          method: "POST",
          headers: {
            "Accept": "application/json",
            "Content-Type": "application/json"
          },
          body: JSON.stringify({
            timestamp: timestamp
          })
        });
        
        if (!response.ok) {
          throw new Error(`Server svarte med ${response.status}`);
        }
        
        const data = await response.json();
        console.log("Tidsynkronisering vellykket:", data);
        alert("Tid synkronisert!");
      } catch (error) {
        console.error("Feil ved synkronisering av tid:", error);
        alert(`Kunne ikke synkronisere tid: ${error.message}`);
      }
    },
    async checkTime() {
      await fetch("http://" + this.serverAddress + "/api/v1/time/check", {
        method: "POST",
        headers: {
          Accept: "application/json",
          "Content-Type": "application/json;charset=utf-8",
          "Access-Control-Allow-Origin": "*",
        },
        body: JSON.stringify({
          timestamp: Math.floor(Date.now() / 1000),
        }),
      });
    },
    async hSearch() {
      await fetch("http://" + this.serverAddress + "/api/v1/time/hsearch", {
        method: "POST",
        headers: {
          Accept: "application/json",
          "Content-Type": "application/json;charset=utf-8",
          "Access-Control-Allow-Origin": "*",
        },
        body: JSON.stringify({
          channel: 0,
        }),
      });
    },
    onSocketOpen(evt) {
      this.socket_ready = true;
    },
    onSocketMessage(evt) {
      var received = JSON.parse(evt.data);
      //console.log("WebSocket melding mottatt:", new Date().toISOString(), received); // Debugging-utskrift
      
      if (!this.lookup.has(received.M)) {
        this.lookup.set(received.M, this.poles.length);
      }
      const id = this.lookup.get(received.M);
      if (this.poles[id] === undefined) {
        this.poles[id] = {
          name: "Pole " + id,
          id: id,
          mac: received.M,
        };
      }

      if (received.K == 0) {
        // ADC values
        this.poles[id].values = received.V;
        this.poles[id].broken = received.B.map((x) =>
          x == 1 ? "#f87979" : "#087979"
        );
        //console.log("Oppdatert ADC-verdier for stolpe", id, this.poles[id].values);
      } else if (received.K == 1) {
        // Sensor breaks
        if (!(received.M in this.breaks)) {
          this.breaks[received.M] = [];
        }
        const br = {
          mac: received.M,
          time: received.T * 1000 + Math.round(received.U / 1000),
          value: received.B,
          filtered: !!received.F  // Sjekk om dette er et filtrert brudd
        };
        this.breaks[received.M].push(br);
        //console.log("Registrert brudd for stolpe", received.M, br);
      } else if (received.K == 2) {
        // Settings. These are sent once, when the pole connects.
        this.poles[id].enabled = received.E.map((x) => (x == 1 ? true : false));
        this.poles[id].offsets = received.O;
        this.poles[id].br_limit = received.B;
        console.log("Mottatt settings for stolpe", id, this.poles[id].enabled);
      } else if (received.K == 3) {
        // Kommando-melding (for logging)
        console.log("Mottatt kommando:", received.cmd);
      } else if (received.K == 4) {
        // Passage detection
        console.log("📣 PASSERING MOTTATT FRA WEBSOCKET:", received);
        if (!this.passages) {
          this.passages = [];
        }
        const passage = {
          mac: received.M,
          time: received.T * 1000 + Math.round(received.U / 1000),
          sensors: received.S
        };
        
        // Beregn tidsdifferanse fra forrige passering hvis det finnes
        if (this.passages.length > 0) {
          const prevPassage = this.passages[this.passages.length - 1];
          passage.timeDiff = passage.time - prevPassage.time;
        } else {
          passage.timeDiff = null;
        }
        
        this.passages.push(passage);
        console.log("📊 PASSAGES ARRAY ETTER OPPDATERING:", JSON.stringify(this.passages));
        //console.log("Passering registrert:", passage);
      }
    },
      
    onSockerError(evt) {
      this.socket_ready = false;
      console.log("On Error");
    },
    
    // Metode for å veksle mellom grensesnitt
    toggleInterface() {
      this.useNewInterface = !this.useNewInterface;
    },
    
    // Nye metoder for tilkoblingsinnstillinger
    toggleConnectionSettings() {
      this.connectionSettings.visible = !this.connectionSettings.visible;
    },
    
    saveConnectionSettings() {
      this.connectionSettings.visible = false;
      // Lagre innstillingene i localStorage for å huske dem
      localStorage.setItem('timergate_useip', this.useIpAddress);
      localStorage.setItem('timergate_ipaddress', this.ipAddress);
      localStorage.setItem('timergate_hostname', this.hostname);
      
      // Koble til på nytt med nye innstillinger
      if (this.socket && this.socket.readyState <= 1) {
        this.socket.close();
      }
      this.init();
    }
  },
  mounted() {
    this.init();
  },
  beforeDestroy() {
    clearInterval(this.interval);
  },
  created() {
    // Last inn lagrede innstillinger hvis de finnes
    if (localStorage.getItem('timergate_useip') !== null) {
      this.useIpAddress = localStorage.getItem('timergate_useip') === 'true';
    }
    if (localStorage.getItem('timergate_ipaddress')) {
      this.ipAddress = localStorage.getItem('timergate_ipaddress');
    }
    if (localStorage.getItem('timergate_hostname')) {
      this.hostname = localStorage.getItem('timergate_hostname');
    }
    
    this.interval = setInterval(() => {
      this.time = Intl.DateTimeFormat("NO", {
        hour: "numeric",
        minute: "numeric",
        second: "numeric",
      }).format();
    }, 1000);
  },
};
</script>

<template>
  <div class="app-wrapper">
    <!-- Knapp for å veksle mellom grensesnitt -->
    <button @click="toggleInterface" class="interface-toggle">
      {{ useNewInterface ? "Bytt til utviklingsvisning" : "Bytt til nytt grensesnitt" }}
    </button>
    
    <!-- Knapp for å vise tilkoblingsinnstillinger -->
    <button @click="toggleConnectionSettings" class="settings-button">
      <span class="settings-icon">⚙️</span>
    </button>
    
    <!-- Tilkoblingsinnstillinger modal -->
    <div v-if="connectionSettings.visible" class="connection-settings-modal">
      <div class="connection-settings-content">
        <h3>Tilkoblingsinnstillinger</h3>
        
        <div class="form-group">
          <label>
            <input type="radio" v-model="useIpAddress" :value="false">
            Bruk hostname
          </label>
          <input 
            type="text" 
            v-model="hostname" 
            :disabled="useIpAddress"
            placeholder="f.eks. timergate.local"
            class="input-field"
          >
        </div>
        
        <div class="form-group">
          <label>
            <input type="radio" v-model="useIpAddress" :value="true">
            Bruk IP-adresse
          </label>
          <input 
            type="text" 
            v-model="ipAddress" 
            :disabled="!useIpAddress"
            placeholder="f.eks. 192.168.4.1"
            class="input-field"
          >
        </div>
        
        <div class="buttons">
          <button @click="saveConnectionSettings" class="save-button">Lagre</button>
          <button @click="connectionSettings.visible = false" class="cancel-button">Avbryt</button>
        </div>
      </div>
    </div>
    
    <!-- Nytt grensesnitt -->
    <main-layout 
      v-if="useNewInterface" 
      :time="time" 
      :poles="poles" 
      :breaks="breaks"
      :passages="passages"
      :serverAddress="serverAddress"
      :sync-time="syncTime"
      :clear-times="clearTimes"
      :h-search="hSearch"
      :check-time="checkTime"
    />
    
    <!-- Eksisterende grensesnitt -->
    <main v-else>
      <h1>Timergate</h1>
      <h3>Current time: {{ time }}</h3>
      <button @click="syncTime()">Sync Time</button>
      <button @click="clearTimes()">Clear Breaks</button>
      <button @click="show_advanced = !show_advanced">Advanced</button>
      <div class="advanced" v-if="show_advanced">
        <h3>Advanced settings</h3>
        <button @click="checkTime()">Check Time Diff</button>
        <button @click="hSearch()">Highpoint search</button>
      </div>
      <div style="width: 100%; height: 250px">
        <Pole
          v-for="pole in poles"
          :key="pole.id"
          :name="pole.name"
          :mac="pole.mac"
          :values="pole.values"
          :broken="pole.broken"
          :offsets="pole.offsets"
          :br_limit="pole.br_limit"
          :show_advanced="show_advanced"
          :serverAddress="serverAddress"
        />
      </div>
      <div>
        <h1>Breaks</h1>
      </div>
      <div
        v-for="(brs, key) in breaks"
        :key="key"
        style="width: 500px; height: 500px; float: left"
      >
        <ul v-for="br in brs.slice().reverse()" :key="br.id">
          <BreakItem
            :id="br.mac"
            :breakTime="br.time"
            :value="br.value"
          ></BreakItem>
        </ul>
      </div>
    </main>
  </div>
</template>

<style>
.app-wrapper {
  font-family: Arial, sans-serif;
  color: #333;
  max-width: 100%;
  margin: 0;
  padding: 0;
  position: relative;
}

.interface-toggle {
  position: fixed;
  top: 10px;
  right: 10px;
  z-index: 1000;
  padding: 8px 12px;
  background: #0078D7;
  color: white;
  border: none;
  border-radius: 4px;
  cursor: pointer;
  font-weight: bold;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
}

.interface-toggle:hover {
  background: #0063b1;
}

.settings-button {
  position: fixed;
  top: 70px; /* Under interface-toggle knappen */
  right: 10px;
  z-index: 1000;
  padding: 8px;
  background: #0078D7;
  color: white;
  border: none;
  border-radius: 4px;
  cursor: pointer;
  font-weight: bold;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
}

.settings-button:hover {
  background: #0063b1;
}

.settings-icon {
  font-size: 16px;
}

.connection-settings-modal {
  position: fixed;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  background-color: rgba(0, 0, 0, 0.5);
  display: flex;
  justify-content: center;
  align-items: center;
  z-index: 2000;
}

.connection-settings-content {
  background-color: white;
  border-radius: 8px;
  padding: 20px;
  width: 90%;
  max-width: 400px;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.2);
}

.connection-settings-content h3 {
  margin-top: 0;
  margin-bottom: 16px;
  color: #333;
  border-bottom: 1px solid #eee;
  padding-bottom: 8px;
}

.form-group {
  margin-bottom: 16px;
}

.form-group label {
  display: block;
  margin-bottom: 8px;
  font-weight: bold;
}

.input-field {
  width: 100%;
  padding: 8px;
  border: 1px solid #ddd;
  border-radius: 4px;
  font-size: 14px;
  margin-top: 6px;
}

.buttons {
  display: flex;
  justify-content: flex-end;
  gap: 8px;
  margin-top: 20px;
}

.save-button {
  background-color: #0078D7;
  color: white;
  border: none;
  border-radius: 4px;
  padding: 8px 16px;
  cursor: pointer;
}

.save-button:hover {
  background-color: #0063b1;
}

.cancel-button {
  background-color: #f5f5f5;
  color: #333;
  border: 1px solid #ddd;
  border-radius: 4px;
  padding: 8px 16px;
  cursor: pointer;
}

.cancel-button:hover {
  background-color: #e5e5e5;
}

h1 {
  color: #333;
}

button {
  padding: 8px 16px;
  margin-right: 10px;
  border: none;
  background-color: #0078D7;
  color: white;
  border-radius: 4px;
  cursor: pointer;
}

button:hover {
  background-color: #0063b1;
}

.advanced {
  margin-top: 15px;
  padding: 15px;
  border: 1px solid #ddd;
  background-color: #f9f9f9;
  border-radius: 4px;
}
</style>