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
      interval: null,
      time: null,
      show_advanced: false,
      
      // Fjernet veksling mellom grensesnitt - alltid bruk MainLayout som standard
      showDevView: false
    };
  },
  computed: {
    // Beregn serveradresse basert på nettleserens URL
    serverAddress() {
      // Hent hostname fra URL-en
      return window.location.hostname;
    },
    
    // Beregn WebSocket URL basert på nettleserens protokoll (HTTP/HTTPS)
    wsProtocol() {
      return window.location.protocol === 'https:' ? 'wss://' : 'ws://';
    },
    
    // Komplett WebSocket URL
    wsUrl() {
      return `${this.wsProtocol}${this.serverAddress}${window.location.port ? ':' + window.location.port : ''}/ws`;
    },
    
    // API base URL
    apiBaseUrl() {
      return `${window.location.protocol}//${this.serverAddress}${window.location.port ? ':' + window.location.port : ''}`;
    }
  },
  methods: {
    init() {
      console.log(`Kobler til WebSocket på ${this.wsUrl}`);
      this.socket = new WebSocket(this.wsUrl);
      this.socket.onopen = this.onSocketOpen;
      this.socket.onmessage = this.onSocketMessage;
      this.socket.onerror = this.onSocketError;
    },
    clearTimes() {
      this.breaks = {};
    },
    async syncTime() {
      console.log(`Synkroniserer tid via ${this.apiBaseUrl}...`);
      try {
        const timestamp = Math.floor(Date.now() / 1000);
        
        const response = await fetch(`${this.apiBaseUrl}/api/v1/time/sync`, {
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
      } catch (error) {
        console.error("Feil ved synkronisering av tid:", error);
      }
    },
    async checkTime() {
      await fetch(`${this.apiBaseUrl}/api/v1/time/check`, {
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
      await fetch(`${this.apiBaseUrl}/api/v1/time/hsearch`, {
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
      console.log("WebSocket tilkobling etablert");
      
      // Automatisk synkronisering av tid når tilkoblingen er opprettet
      this.syncTime();
    },
    onSocketMessage(evt) {
      var received = JSON.parse(evt.data);
      
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
          filtered: !!received.F  // Sjekk om dette er et filtrert brudd
        };
        this.breaks[received.M].push(br);
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
      }
    },
    onSocketError(evt) {
      this.socket_ready = false;
      console.error("WebSocket tilkoblingsfeil:", evt);
      
      // Forsøk å koble til på nytt etter 5 sekunder
      setTimeout(() => {
        console.log("Forsøker å koble til WebSocket på nytt...");
        this.init();
      }, 5000);
    },
    
    // Håndter visning av utviklingsgrensesnittet
    toggleDevView() {
      this.showDevView = !this.showDevView;
    }
  },
  mounted() {
    this.init();
  },
  beforeDestroy() {
    clearInterval(this.interval);
    
    // Lukk WebSocket-tilkoblingen
    if (this.socket) {
      this.socket.close();
    }
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
    <!-- Nytt grensesnitt som standard -->
    <main-layout 
      v-if="!showDevView" 
      :time="time" 
      :poles="poles" 
      :breaks="breaks"
      :passages="passages"
      :serverAddress="serverAddress"
      :sync-time="syncTime"
      :clear-times="clearTimes"
      :h-search="hSearch"
      :check-time="checkTime"
      @toggle-dev="toggleDevView"
    />
    
    <!-- Utviklingsgrensesnitt når trengt -->
    <main v-else>
      <h1>Timergate (Utviklingsvisning)</h1>
      <button @click="toggleDevView" class="back-button">Tilbake til hovedgrensesnitt</button>
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

.back-button {
  margin-bottom: 15px;
  background-color: #0078D7;
  color: white;
  border: none;
  border-radius: 4px;
  padding: 8px 12px;
  cursor: pointer;
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