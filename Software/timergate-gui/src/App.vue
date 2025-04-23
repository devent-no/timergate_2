<script>
import BreakItem from "./components/BreakItem.vue";
import Pole from "./components/Pole.vue";
import MainLayout from "./layouts/MainLayout.vue";

export default {
  components: {
    BreakItem,
    Pole,
    MainLayout
  },
  data() {
    return {
      count: 0,
      poles: [],
      lookup: new Map(),
      breaks: {},
      settings: {},
      socket: null,
      socket_ready: false,
      hostname: "timergate.local",
      interval: null,
      time: null,
      show_advanced: false,
      
      // Ny status for å veksle mellom grensesnitt
      useNewInterface: false
    };
  },
  methods: {
    init() {
      this.socket = new WebSocket("ws://" + this.hostname + "/ws");
      this.socket.onopen = this.onSocketOpen;
      this.socket.onmessage = this.onSocketMessage;
      this.socket.onerror = this.onSockerError;
    },
    clearTimes() {
      this.breaks = {};
    },
    async syncTime() {
      await fetch("http://" + this.hostname + "/api/v1/time/sync", {
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
    async checkTime() {
      await fetch("http://" + this.hostname + "/api/v1/time/check", {
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
      await fetch("http://" + this.hostname + "/api/v1/time/hsearch", {
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

      // Sjekk om dette er en filtrert melding
      const isFiltered = received.F === true;

      // For det nye GUI, bruk kun filtrerte meldinger
      if (this.useNewInterface && !isFiltered && received.K == 1) {
        // Ignorer ufiltrerte brudd-meldinger i det nye grensesnittet
        return;
      }


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
      } else if (received.K == 1) {
        // Sensor breaks
        if (!(received.M in this.breaks)) {
          this.breaks[received.M] = [];
        }
        const br = {
          mac: received.M,
          time: received.T * 1000 + Math.round(received.U / 1000),
          value: received.B,
        };
        this.breaks[received.M].push(br);
      } else if (received.K == 2) {
        // Settings. These are sent once, when the pole connects.
        this.poles[id].enabled = received.E.map((x) => (x == 1 ? true : false));
        this.poles[id].offsets = received.O;
        this.poles[id].br_limit = received.B;
        console.log(this.poles[id].enabled);
      }
    },
    onSockerError(evt) {
      this.socket_ready = false;
      console.log("On Error");
    },
    
    // Ny metode for å veksle mellom grensesnitt
    toggleInterface() {
      this.useNewInterface = !this.useNewInterface;
    }
  },
  mounted() {
    this.init();
  },
  beforeDestroy() {
    clearInterval(this.interval);
  },
  created() {
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
    <!-- Vekslerknapp -->
    <button @click="toggleInterface" class="interface-toggle">
      {{ useNewInterface ? "Bytt til utviklingsvisning" : "Bytt til nytt grensesnitt" }}
    </button>
    
    <!-- Nytt grensesnitt -->
    <main-layout 
      v-if="useNewInterface" 
      :time="time" 
      :poles="poles" 
      :breaks="breaks"
      :hostname="hostname"
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