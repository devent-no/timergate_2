<script>
import Dashboard from "../views/Dashboard.vue";
import TimerView from "../views/TimerView.vue";
import DevicesView from "../views/DevicesView.vue";
import ConfigView from "../views/ConfigView.vue";
import LogView from "../views/LogView.vue";

export default {
  components: {
    Dashboard,
    TimerView,
    DevicesView,
    ConfigView,
    LogView
  },
  props: {
    time: {
      type: String,
      default: null
    },
    poles: {
      type: Array,
      default: () => []
    },
    breaks: {
      type: Object,
      default: () => ({})
    },
    passages: {
      type: Array,
      default: () => []
    },
    // Serveradresse-prop
    serverAddress: {
      type: String,
      default: "timergate.local"
    },
    // Siste synkroniseringstid
    lastSyncTime: {
      type: String,
      default: null
    }
  },
  data() {
    return {
      currentView: "dashboard", // Standard visning
      // Nye data for discovery og pairing
      discoveredPoles: [],
      pairedPoles: [],
      systemId: "",
      signalData: {}
    };
  },

  methods: {
    navigateTo(view) {
      this.currentView = view;
    },
    
    // Handler for når hostname endres i ConfigView
    onHostnameChanged(newHostname) {
      console.log("MainLayout: Hostname endret til", newHostname);
      this.$emit('hostname-changed', newHostname);
    },
    
    // Handler for når tid synkroniseres i Dashboard
    onTimeSynced(syncTime) {
      console.log("MainLayout: Tid synkronisert på", syncTime);
      this.$emit('time-synced', syncTime);
    },
    
    // Bytt til utviklingsvisning
    switchToDevView() {
      this.$emit('toggle-dev');
    },
    
    // Nye metoder for å laste discovery og pairing data
    async loadDiscoveryData() {
      try {
        // Last system info
        const systemResponse = await fetch(`http://${this.serverAddress}/api/v1/system/id`);
        const systemData = await systemResponse.json();
        if (systemData.status === 'success') {
          this.systemId = systemData.system_id || '';
        }
        
        // Last discovered poles
        const discoveredResponse = await fetch(`http://${this.serverAddress}/api/v1/poles/discovered`);
        const discoveredData = await discoveredResponse.json();
        if (discoveredData.status === 'success') {
          this.discoveredPoles = discoveredData.poles || [];
        }
        
        // Last paired poles
        const pairedResponse = await fetch(`http://${this.serverAddress}/api/v1/poles/paired`);
        const pairedData = await pairedResponse.json();
        if (pairedData.status === 'success') {
          this.pairedPoles = pairedData.poles || [];
        }
        
        // LEGG TIL DENNE DELEN:
        // Last signal quality
        await this.loadSignalQuality();
        
      } catch (error) {
        console.error('Feil ved lasting av discovery data:', error);
      }
    },

    // LEGG TIL DENNE NYE METODEN:
    async loadSignalQuality() {
      try {
        const response = await fetch(`http://${this.serverAddress}/api/v1/signal/quality`);
        const data = await response.json();
        
        if (data.status === 'success') {
          const signalMap = {};
          data.signal_data.forEach(signal => {
            const normalizedMac = signal.mac.toLowerCase();
            signalMap[signal.mac] = signal;
            signalMap[normalizedMac] = signal;
          });
          this.signalData = signalMap;
          console.log('📶 MainLayout: Signalkvalitet lastet', Object.keys(signalMap));
        }
      } catch (error) {
        console.error('Feil ved lasting av signalkvalitet:', error);
      }
    },


  },

  mounted() {
    // Last discovery data ved oppstart
    this.loadDiscoveryData();
    
    // Sett opp periodisk oppdatering
    this.discoveryInterval = setInterval(() => {
      this.loadDiscoveryData();
    }, 10000); // Oppdater hvert 10. sekund
  },

  beforeDestroy() {
    if (this.discoveryInterval) {
      clearInterval(this.discoveryInterval);
    }


  }

};
</script>

<template>
  <div class="app-container">
    <!-- Header med logo og navigasjon -->
    <header class="app-header">
      <div class="logo">Timergate</div>
      <nav class="main-nav">
        <button 
          @click="navigateTo('dashboard')" 
          :class="{ active: currentView === 'dashboard' }"
        >
          <span class="icon">⏱️</span>
          <span class="label">Dashboard</span>
        </button>
        
        <button 
          @click="navigateTo('timer')" 
          :class="{ active: currentView === 'timer' }"
        >
          <span class="icon">🕒</span>
          <span class="label">Timer</span>
        </button>
        
        <button 
          @click="navigateTo('devices')" 
          :class="{ active: currentView === 'devices' }"
        >
          <span class="icon">📡</span>
          <span class="label">Enheter</span>
        </button>
        
        <button 
          @click="navigateTo('config')" 
          :class="{ active: currentView === 'config' }"
        >
          <span class="icon">⚙️</span>
          <span class="label">Konfigurering</span>
        </button>
        
        <button 
          @click="navigateTo('log')" 
          :class="{ active: currentView === 'log' }"
        >
          <span class="icon">📋</span>
          <span class="label">Logg</span>
        </button>
        
        <!-- Ny knapp for utviklingsvisning -->
        <button 
          @click="switchToDevView()" 
          class="dev-button"
        >
          <span class="icon">🔧</span>
          <span class="label">Dev</span>
        </button>
      </nav>
    </header>
    
    <!-- Hovedinnhold basert på valgt visning -->
    <main class="app-content">
      <dashboard 
        v-if="currentView === 'dashboard'" 
        :time="time" 
        :poles="poles" 
        :breaks="breaks" 
        :serverAddress="serverAddress"
        :lastSyncTime="lastSyncTime"
        @time-synced="onTimeSynced"
      />
      <timer-view 
        v-else-if="currentView === 'timer'" 
        :time="time" 
        :poles="poles" 
        :breaks="breaks" 
        :passages="passages" 
        :serverAddress="serverAddress"
      />
      <devices-view 
        v-else-if="currentView === 'devices'" 
        :poles="poles" 
        :serverAddress="serverAddress"
        :discoveredPoles="discoveredPoles"
        :pairedPoles="pairedPoles"
        :systemId="systemId"
        :signalData="signalData"
        :currentView="currentView"
      />
      <config-view 
        v-else-if="currentView === 'config'" 
        :serverAddress="serverAddress"
        @hostname-changed="onHostnameChanged"
      />
      <log-view 
        v-else-if="currentView === 'log'" 
        :breaks="breaks" 
        :passages="passages" 
        :serverAddress="serverAddress"
      />
    </main>
  </div>
</template>

<style scoped>
.app-container {
  display: flex;
  flex-direction: column;
  height: 100vh;
  font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
}

.app-header {
  display: flex;
  align-items: center;
  padding: 16px 24px;
  background-color: #0078D7;
  color: white;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
}

.logo {
  font-size: 24px;
  font-weight: bold;
  margin-right: 40px;
}

.main-nav {
  display: flex;
  gap: 8px;
}

.main-nav button {
  display: flex;
  flex-direction: column;
  align-items: center;
  background: none;
  border: none;
  color: rgba(255, 255, 255, 0.8);
  cursor: pointer;
  padding: 8px 16px;
  border-radius: 4px;
  transition: all 0.2s;
}

.main-nav button:hover {
  background-color: rgba(255, 255, 255, 0.1);
}

.main-nav button.active {
  background-color: rgba(255, 255, 255, 0.2);
  color: white;
}

.main-nav .icon {
  font-size: 24px;
  margin-bottom: 4px;
}

.main-nav .label {
  font-size: 12px;
}

/* Spesiell styling for Dev-knappen */
.main-nav .dev-button {
  margin-left: auto; /* Skyv til høyre */
  background-color: rgba(0, 0, 0, 0.1);
}

.app-content {
  flex: 1;
  padding: 24px;
  background-color: #f5f5f5;
  overflow-y: auto;
}
</style>