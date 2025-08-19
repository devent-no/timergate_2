<template>
  <div class="app-container">
    <!-- Header med logo, navigasjon og målestolpe-status -->
    <header class="app-header">
      <div class="header-left">
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
          
          <!-- Utviklingsknapp -->
          <button 
            @click="switchToDevView()" 
            class="dev-button"
          >
            <span class="icon">🔧</span>
            <span class="label">Dev</span>
          </button>
        </nav>
      </div>
      
      <!-- NYTT: Målestolpe-status i header -->
      <div class="header-right">
        <pole-status-indicator 
          ref="poleStatusIndicator"
          :server-address="serverAddress"
          :poles="poles"
          class="header-pole-status"
        />
      </div>
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

<script>
import Dashboard from "../views/Dashboard.vue";
import TimerView from "../views/TimerView.vue";
import DevicesView from "../views/DevicesView.vue";
import ConfigView from "../views/ConfigView.vue";
import LogView from "../views/LogView.vue";
import PoleStatusIndicator from "../components/PoleStatusIndicator.vue";

export default {
  components: {
    Dashboard,
    TimerView,
    DevicesView,
    ConfigView,
    LogView,
    PoleStatusIndicator
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
  watch: {
    // FJERNET: breaks watcher (erstattet med direkte K=1 forwarding)
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
    
    // NYTT: Motta K=1 data fra App.vue og send til PoleStatusIndicator
    updatePoleStatusFromBreak(mac, sensorId, breakState) {
      if (this.$refs.poleStatusIndicator) {
        this.$refs.poleStatusIndicator.updateSensorFromBreak(mac, sensorId, breakState);
        console.log(`MainLayout: Videresendt sensor-break til PoleStatusIndicator: MAC=${mac}, Sensor=${sensorId}, State=${breakState}`);
      }
    },
    
    // Nye metoder for å laste discovery og pairing data
    async loadDiscoveryData() {
      try {
        // Last system info
        const systemResponse = await fetch(`http://${this.serverAddress}/api/v1/system/id`);
        const systemData = await systemResponse.json();
        if (systemData.status === 'success') {
          this.systemId = systemData.system_id;
        }
        
        // Last discovered poles
        const discoveredResponse = await fetch(`http://${this.serverAddress}/api/v1/poles/discovered`);
        const discoveredData = await discoveredResponse.json();
        if (discoveredData.status === 'success') {
          this.discoveredPoles = discoveredData.poles;
        }
        
        // Last paired poles
        const pairedResponse = await fetch(`http://${this.serverAddress}/api/v1/poles/paired`);
        const pairedData = await pairedResponse.json();
        if (pairedData.status === 'success') {
          this.pairedPoles = pairedData.poles;
        }
        
        // Last signal data
        const signalResponse = await fetch(`http://${this.serverAddress}/api/v1/signal/quality`);
        const signalData = await signalResponse.json();
        if (signalData.status === 'success') {
          // Konverter array til objekt med MAC som nøkkel for lettere oppslag
          this.signalData = signalData.signal_data.reduce((acc, signal) => {
            acc[signal.mac] = signal;
            return acc;
          }, {});
        }
        
      } catch (error) {
        console.error('Feil ved lasting av discovery data:', error);
      }
    }
  },
  
  mounted() {
    // Last discovery og pairing data når komponenten mountes
    this.loadDiscoveryData();
    
    // Oppdater discovery data hvert 10. sekund
    this.discoveryInterval = setInterval(() => {
      this.loadDiscoveryData();
    }, 10000);
  },
  
  beforeUnmount() {
    if (this.discoveryInterval) {
      clearInterval(this.discoveryInterval);
    }
  }
};
</script>

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
  justify-content: space-between;
  padding: 16px 24px;
  background-color: #0078D7;
  color: white;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
}

.header-left {
  display: flex;
  align-items: center;
}

.header-right {
  display: flex;
  align-items: center;
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
  margin-left: auto;
  background-color: rgba(0, 0, 0, 0.1);
}

/* NYTT: Styling for målestolpe-status i header */
.header-pole-status {
  background-color: rgba(255, 255, 255, 0.1);
  border: 1px solid rgba(255, 255, 255, 0.2);
  border-radius: 8px;
  max-width: 300px;
}

/* Override pole status colors for dark header */
.header-pole-status :deep(.pole-indicator.signal-excellent) {
  background-color: rgba(76, 175, 80, 0.2);
  border-left-color: #4caf50;
  color: rgba(255, 255, 255, 0.9);
}

.header-pole-status :deep(.pole-indicator.signal-good) {
  background-color: rgba(255, 152, 0, 0.2);
  border-left-color: #ff9800;
  color: rgba(255, 255, 255, 0.9);
}

.header-pole-status :deep(.pole-indicator.signal-poor) {
  background-color: rgba(244, 67, 54, 0.2);
  border-left-color: #f44336;
  color: rgba(255, 255, 255, 0.9);
}

.header-pole-status :deep(.pole-indicator.signal-critical) {
  background-color: rgba(211, 47, 47, 0.2);
  border-left-color: #d32f2f;
  color: rgba(255, 255, 255, 0.9);
}

.header-pole-status :deep(.pole-status-header) {
  background-color: rgba(255, 255, 255, 0.1);
  color: rgba(255, 255, 255, 0.9);
}

.header-pole-status :deep(.status-title) {
  color: rgba(255, 255, 255, 0.9);
}

.header-pole-status :deep(.refresh-btn:hover:not(:disabled)) {
  background-color: rgba(255, 255, 255, 0.1);
}

.app-content {
  flex: 1;
  padding: 24px;
  background-color: #f5f5f5;
  overflow-y: auto;
}

/* Responsiv design for mindre skjermer */
@media (max-width: 1024px) {
  .app-header {
    flex-direction: column;
    gap: 16px;
    padding: 12px 16px;
  }
  
  .header-left {
    width: 100%;
    justify-content: space-between;
  }
  
  .header-right {
    width: 100%;
    justify-content: center;
  }
  
  .header-pole-status {
    max-width: none;
    width: 100%;
  }
}

@media (max-width: 768px) {
  .logo {
    margin-right: 20px;
  }
  
  .main-nav {
    gap: 4px;
  }
  
  .main-nav button {
    padding: 6px 12px;
  }
  
  .main-nav .icon {
    font-size: 20px;
  }
  
  .main-nav .label {
    font-size: 11px;
  }
}
</style>