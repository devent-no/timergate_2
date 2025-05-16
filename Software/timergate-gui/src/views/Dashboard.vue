<template>
  <div>
    <h1>Timergate Dashboard</h1>
    
    <div class="dashboard-grid">
      <!-- Tidspanel -->
      <div class="dashboard-panel time-panel">
        <h2>Gjeldende tid</h2>
        <div class="current-time">{{ time || "00:00" }}</div>
      </div>
      
      <!-- Statusoversikt -->
      <div class="dashboard-panel status-panel">
        <h2>Systemstatus</h2>
        <div class="status-item">
          <span class="status-label">Tilkoblede målestolper:</span>
          <span class="status-value">{{ connectedPoles }}</span>
        </div>
        <div class="status-item">
          <span class="status-label">Aktive sensorer:</span>
          <span class="status-value">{{ activeSensors }}</span>
        </div>
        <div class="status-item">
          <span class="status-label">Tidspunkt for siste passering:</span>
          <span class="status-value">{{ lastBreakTime }}</span>
        </div>
      </div>
      
      <!-- Hurtighandlinger -->
      <div class="dashboard-panel actions-panel">
        <h2>Hurtighandlinger</h2>
        <div class="action-buttons">
          <button 
            @click="syncTime" 
            class="action-button sync"
            :disabled="isSyncing"
          >
            {{ isSyncing ? 'Synkroniserer...' : 'Synkroniser tid' }}
          </button>
          <button 
            @click="calibrateSensors" 
            class="action-button calibrate"
            :disabled="isCalibrating"
          >
            {{ isCalibrating ? 'Kalibrerer...' : 'Kalibrer sensorer' }}
          </button>
          <button @click="resetTimer" class="action-button reset">Nullstill timer</button>
        </div>
        
        <!-- Statusmeldinger -->
        <div v-if="statusMessage" class="status-message" :class="statusType">
          {{ statusMessage }}
        </div>
      </div>
    </div>
  </div>
</template>

<script>
export default {
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
    // Legg til serverAddress-prop
    serverAddress: {
      type: String,
      default: "timergate.local"
    }
  },
  data() {
    return {
      isSyncing: false,
      isCalibrating: false,
      statusMessage: "",
      statusType: "success"
    };
  },
  computed: {
    connectedPoles() {
      return this.poles.length;
    },
    activeSensors() {
      let count = 0;
      this.poles.forEach(pole => {
        if (pole.values) {
          count += pole.values.filter(v => v > 0).length;
        }
      });
      return count;
    },
    lastBreakTime() {
      let lastTime = null;
      for (const mac in this.breaks) {
        const macBreaks = this.breaks[mac];
        if (macBreaks && macBreaks.length > 0) {
          const time = macBreaks[0].time;
          if (!lastTime || time > lastTime) {
            lastTime = time;
          }
        }
      }
      if (lastTime) {
        const date = new Date(lastTime);
        return Intl.DateTimeFormat("NO", {
          hour: "numeric",
          minute: "numeric",
          second: "numeric"
        }).format(date);
      }
      return "Ingen";
    }
  },
  methods: {
    // Forbedret synkroniseringsfunksjon
    async syncTime() {
      if (this.isSyncing) return;
      
      this.isSyncing = true;
      this.statusMessage = "";
      this.statusType = "success";
      
      try {
        console.log(`Dashboard: Synkroniserer tid via ${this.serverAddress}...`);
        const timestamp = Math.floor(Date.now() / 1000);
        
        const response = await fetch(`http://${this.serverAddress}/api/v1/time/sync`, {
          method: "POST",
          headers: {
            "Accept": "application/json",
            "Content-Type": "application/json",
            "Access-Control-Allow-Origin": "*"
          },
          body: JSON.stringify({
            timestamp: timestamp
          })
        });
        
        if (!response.ok) {
          throw new Error(`Server svarte med ${response.status}: ${response.statusText}`);
        }
        
        const data = await response.json();
        this.statusMessage = "Tid synkronisert!";
        this.statusType = "success";
        console.log("Tidsynkronisering vellykket:", data);
      } catch (error) {
        console.error("Feil ved synkronisering av tid:", error);
        this.statusMessage = `Kunne ikke synkronisere tid: ${error.message}`;
        this.statusType = "error";
      } finally {
        this.isSyncing = false;
        
        // Fjern statusmelding etter 3 sekunder
        setTimeout(() => {
          this.statusMessage = "";
        }, 3000);
      }
    },
    
    // Forbedret kalibreringsfunksjon
    async calibrateSensors() {
      if (this.isCalibrating) return;
      
      this.isCalibrating = true;
      this.statusMessage = "";
      this.statusType = "success";
      
      try {
        console.log(`Kalibrerer sensorer via ${this.serverAddress}...`);
        
        const response = await fetch(`http://${this.serverAddress}/api/v1/time/hsearch`, {
          method: "POST",
          headers: {
            "Accept": "application/json",
            "Content-Type": "application/json",
            "Access-Control-Allow-Origin": "*"
          },
          body: JSON.stringify({
            channel: 0
          })
        });
        
        if (!response.ok) {
          throw new Error(`Server svarte med ${response.status}: ${response.statusText}`);
        }
        
        const data = await response.json();
        this.statusMessage = "Kalibrering startet!";
        this.statusType = "success";
        console.log("Kalibrering startet:", data);
      } catch (error) {
        console.error("Feil ved kalibrering av sensorer:", error);
        this.statusMessage = `Kunne ikke starte kalibrering: ${error.message}`;
        this.statusType = "error";
      } finally {
        this.isCalibrating = false;
        
        // Fjern statusmelding etter 3 sekunder
        setTimeout(() => {
          this.statusMessage = "";
        }, 3000);
      }
    },
    
    async resetTimer() {
      this.statusMessage = "Timer nullstilt";
      this.statusType = "success";
      
      // Fjern statusmelding etter 3 sekunder
      setTimeout(() => {
        this.statusMessage = "";
      }, 3000);
    }
  }
};
</script>

<style scoped>
h1 {
  margin-bottom: 24px;
  color: #333;
}

.dashboard-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 24px;
}

.dashboard-panel {
  background-color: white;
  border-radius: 8px;
  padding: 20px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.05);
}

.dashboard-panel h2 {
  margin-top: 0;
  margin-bottom: 16px;
  color: #555;
  font-size: 18px;
  border-bottom: 1px solid #eee;
  padding-bottom: 8px;
}

.time-panel .current-time {
  font-size: 48px;
  font-weight: bold;
  text-align: center;
  margin: 20px 0;
  color: #0078D7;
}

.status-item {
  display: flex;
  justify-content: space-between;
  margin-bottom: 12px;
  padding-bottom: 12px;
  border-bottom: 1px solid #f5f5f5;
}

.status-item:last-child {
  border-bottom: none;
}

.status-label {
  color: #555;
}

.status-value {
  font-weight: bold;
  color: #333;
}

.action-buttons {
  display: flex;
  flex-direction: column;
  gap: 12px;
}

.action-button {
  padding: 12px;
  border: none;
  border-radius: 4px;
  font-weight: bold;
  cursor: pointer;
  transition: background-color 0.2s;
}

.action-button:disabled {
  opacity: 0.6;
  cursor: not-allowed;
}

.action-button.sync {
  background-color: #0078D7;
  color: white;
}

.action-button.calibrate {
  background-color: #28a745;
  color: white;
}

.action-button.reset {
  background-color: #dc3545;
  color: white;
}

.action-button:hover:not(:disabled) {
  opacity: 0.9;
}

.status-message {
  margin-top: 16px;
  padding: 10px;
  border-radius: 4px;
  text-align: center;
  font-weight: bold;
}

.status-message.success {
  background-color: #d4edda;
  color: #155724;
  border: 1px solid #c3e6cb;
}

.status-message.error {
  background-color: #f8d7da;
  color: #721c24;
  border: 1px solid #f5c6cb;
}
</style>