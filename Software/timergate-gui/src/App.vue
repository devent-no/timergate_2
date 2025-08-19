<template>
  <div class="app-wrapper">
    <!-- Bruk MainLayout som standard -->
    <MainLayout 
      v-if="!showDevView"
      :time="time" 
      :poles="poles" 
      :breaks="breaks" 
      :passages="passages" 
      :serverAddress="serverAddress"
      :lastSyncTime="lastSyncTime"
      ref="mainLayout"
      @hostname-changed="onHostnameChanged"
      @toggle-dev="toggleDevView"
      @time-synced="onTimeSynced"
    />
    
    <!-- Utviklingsgrensesnitt (legacy) -->
    <main v-else class="app-wrapper">
      <button class="back-button" @click="toggleDevView">← Tilbake til hovedgrensesnitt</button>
      
      <h1>Poles</h1>
      <div>
        <span>Time: {{ time }}</span>
        <button @click="clearTimes()">Clear times</button>
        <button @click="syncTime()">Sync time</button>
        <button @click="checkTime()">Check time</button>
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
      lastSyncTime: null,
      
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
        this.lastSyncTime = new Date().toLocaleTimeString();
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
        // 🔍 DETALJERT LOGGING AV K=1 MELDINGER
        console.log("🔍 K=1 RAW DATA:", {
          fullMessage: received,
          mac: received.M,
          timestamp_sec: received.T,
          timestamp_usec: received.U,
          B_field: received.B,
          S_field: received.S,  // ← NYTT: Legg til S-felt i logging
          F_field: received.F,
          otherFields: Object.keys(received).filter(key => !['M','K','T','U','B','S','F'].includes(key))
        });
        
        // FORBEDRET: Bruk faktisk break_state fra S-feltet
        if (!(received.M in this.breaks)) {
          this.breaks[received.M] = [];
        }
        
        // FIKSET: Bruk korrekte felter
        const sensorId = received.B;      // sensor_id (0-6)
        const breakState = received.S !== undefined ? received.S : 1; // break_state (1=brudd, 0=gjenopprettet)
        
        const br = {
          mac: received.M,
          time: received.T * 1000 + Math.round(received.U / 1000),
          value: sensorId,
          filtered: !!received.F  // Sjekk om dette er et filtrert brudd
        };
        this.breaks[received.M].push(br);
        
        // FORBEDRET: Send både brudd og gjenopprettelse til målestolpe-status komponent
        console.log(`🔄 K=1 MOTTATT: MAC=${received.M}, sensor_id=${sensorId}, break_state=${breakState}`);
        this.updatePoleStatusFromBreak(received.M, sensorId, breakState);
        
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
        console.log("🔣 PASSERING MOTTATT FRA WEBSOCKET:", received);
        if (!this.passages) {
          this.passages = [];
        }
        const passage = {
          mac: received.M,
          time: received.T * 1000 + Math.round(received.U / 1000),
          sensors: received.S,
          T: received.T,    // ← LEGG TIL: Server timestamp sekunder
          U: received.U     // ← LEGG TIL: Server timestamp mikrosekunder
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
    
    // FORBEDRET: Send K=1 sensor break-data til målestolpe-status komponent
    updatePoleStatusFromBreak(mac, sensorId, breakState) {
      // Send til MainLayout som videreformidler til PoleStatusIndicator
      if (this.$refs.mainLayout) {
        this.$refs.mainLayout.updatePoleStatusFromBreak(mac, sensorId, breakState);
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
    
    // Håndter hostname-endring fra ConfigView
    onHostnameChanged(newHostname) {
      console.log("App: Hostname endret til", newHostname);
      // Her kan du implementere logikk for å håndtere hostname-endring
    },
    
    // Håndter tidsynkronisering
    onTimeSynced(syncTime) {
      this.lastSyncTime = syncTime;
    },
    
    // Veksle til/fra utviklingsvisning
    toggleDevView() {
      this.showDevView = !this.showDevView;
      console.log("Vekslet til", this.showDevView ? "utviklingsvisning" : "hovedgrensesnitt");
    }
  },
  mounted() {
    this.init();
    
    // Sett opp intervall for å oppdatere tid
    this.interval = setInterval(() => {
      this.time = new Date().toLocaleTimeString();
    }, 1000);
  },
  beforeUnmount() {
    if (this.interval) {
      clearInterval(this.interval);
    }
    if (this.socket) {
      this.socket.close();
    }
  }
};
</script>

<style scoped>
.app-wrapper {
  display: flex;
  flex-direction: column;
  height: 100vh;
}

.back-button {
  margin: 10px;
  padding: 8px 16px;
  background-color: #007bff;
  color: white;
  border: none;
  border-radius: 4px;
  cursor: pointer;
}

.back-button:hover {
  background-color: #0056b3;
}
</style>