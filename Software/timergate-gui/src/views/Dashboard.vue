<template>
  <div>
    <h1>Timergate Dashboard</h1>
    
    <div class="dashboard-grid">
      <!-- Tidspanel -->
      <div class="dashboard-panel time-panel">
        <h2>Gjeldende tid</h2>
        <div class="current-time">{{ time || "00:00" }}</div>
        <div v-if="lastSyncTime" class="sync-status">
          <span class="sync-icon">🔄</span> Sist synkronisert: {{ lastSyncTime }}
        </div>
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
        <div class="status-item">
          <span class="status-label">Server:</span>
          <span class="status-value">{{ serverAddress }}</span>
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
          <button 
            @click="resetTimer" 
            class="action-button reset"
          >
            Nullstill timer
          </button>
        </div>
        
        <!-- Statusmelding -->
        <div 
          v-if="statusMessage" 
          class="status-message"
          :class="statusType"
        >
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
    serverAddress: {
      type: String,
      default: "timergate.local"
    },
    lastSyncTime: {
      type: String,
      default: null
    }
  },
  data() {
    return {
      isSyncing: false,
      isCalibrating: false,
      statusMessage: "",
      statusType: ""
    };
  },
  computed: {
    connectedPoles() {
      return this.poles.length;
    },
    
    activeSensors() {
      let totalActive = 0;
      this.poles.forEach(pole => {
        if (pole.broken) {
          // Regn antall aktive sensorer basert på broken array
          totalActive += pole.broken.filter(broken => !broken).length;
        }
      });
      return totalActive;
    },
    
    lastBreakTime() {
      // Finn siste break-tid fra breaks objektet
      let latestTime = null;
      Object.values(this.breaks).forEach(breakArray => {
        if (breakArray.length > 0) {
          const lastBreak = breakArray[breakArray.length - 1];
          if (!latestTime || lastBreak.time > latestTime) {
            latestTime = lastBreak.time;
          }
        }
      });
      
      if (latestTime) {
        return new Date(latestTime * 1000).toLocaleTimeString();
      }
      return "Ingen registrert";
    }
  },
  
  methods: {
    async syncTime() {
      this.isSyncing = true;
      this.statusMessage = "";
      
      try {
        const timestamp = Math.floor(Date.now() / 1000);
        
        const response = await fetch(`http://${this.serverAddress}/api/v1/time/sync`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({ timestamp })
        });
        
        const data = await response.json();
        
        if (data.status === 'success') {
          this.statusMessage = "Tid synkronisert med alle målestolper";
          this.statusType = "success";
          this.$emit('time-synced', new Date().toLocaleTimeString());
        } else {
          throw new Error(data.message || 'Ukjent feil');
        }
      } catch (error) {
        console.error("Feil ved synkronisering av tid:", error);
        this.statusMessage = `Kunne ikke synkronisere tid: ${error.message}`;
        this.statusType = "error";
      } finally {
        this.isSyncing = false;
        
        // Fjern statusmelding etter 5 sekunder
        setTimeout(() => {
          if (this.statusType === "success") {
            this.statusMessage = "";
          }
        }, 5000);
      }
    },
    
    async calibrateSensors() {
      this.isCalibrating = true;
      this.statusMessage = "";
      
      try {
        const response = await fetch(`http://${this.serverAddress}/api/v1/time/hsearch`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({ channel: 1 })
        });
        
        const data = await response.json();
        
        if (data.status === 'success') {
          this.statusMessage = "Sensorkalibrering startet på alle målestolper";
          this.statusType = "success";
          console.log("Kalibrering startet:", data);
        } else {
          throw new Error(data.message || 'Ukjent feil');
        }
      } catch (error) {
        console.error("Feil ved kalibrering av sensorer:", error);
        this.statusMessage = `Kunne ikke starte kalibrering: ${error.message}`;
        this.statusType = "error";
      } finally {
        this.isCalibrating = false;
        
        // Fjern statusmelding etter 3 sekunder
        setTimeout(() => {
          if (this.statusType === "success") {
            this.statusMessage = "";
          }
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

.sync-status {
  text-align: center;
  color: #555;
  font-size: 14px;
  margin-top: -15px;
  padding-bottom: 10px;
}

.sync-icon {
  margin-right: 4px;
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

.status-message.pending {
  background-color: #fff3cd;
  color: #856404;
  border: 1px solid #ffeeba;
}

/* Responsiv layout for dashboard */
@media (max-width: 1200px) {
  .dashboard-grid {
    grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
  }
}

@media (max-width: 768px) {
  .dashboard-grid {
    grid-template-columns: 1fr;
  }
  
  .time-panel .current-time {
    font-size: 36px;
  }
}
</style>