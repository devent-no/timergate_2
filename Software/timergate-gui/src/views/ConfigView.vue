<script>
export default {
  props: {
    // Behold serverAddress-prop for konsistens med andre komponenter
    serverAddress: {
      type: String,
      default: ""
    }
  },
  data() {
    return {
      debounceMs: 1000,
      minSensors: 2,
      timeoutMs: 250,
      isSaving: false,
      saveMessage: "",
      saveStatus: "",
      // Nye data for system info
      systemId: "",
      systemName: "",
      discoveryActive: false,
      isLoadingSystemInfo: false
    };
  },
  computed: {
    // Beregnet API base URL
    apiBaseUrl() {
      return `${window.location.protocol}//${this.serverAddress || window.location.host}`;
    },
    
    // Nye computed properties for reactivity
    displaySystemId() {
      return this.systemId || 'Laster...';
    },
    
    displaySystemName() {
      return this.systemName || '';
    }
  },


  mounted() {
    this.fetchCurrentConfig();
    this.fetchSystemInfo();
  },
  
  methods: {
    async fetchCurrentConfig() {
      try {
        const response = await fetch(`${this.apiBaseUrl}/api/v1/config/passage`);
        const data = await response.json();
        
        if (data.status === "success" && data.config) {
          this.debounceMs = data.config.debounce_ms;
          this.minSensors = data.config.min_sensors;
          this.timeoutMs = data.config.timeout_ms;
        }
      } catch (error) {
        console.error("Feil ved henting av passering-konfigurasjon:", error);
      }
    },

    async fetchSystemInfo() {
      this.isLoadingSystemInfo = true;
      try {
        const response = await fetch(`${this.apiBaseUrl}/api/v1/system/id`);
        const data = await response.json();
        
        if (data.status === "success") {
          this.systemId = data.system_id || "";
          this.systemName = data.system_name || "Timergate System";
          this.discoveryActive = data.discovery_active || false;
          console.log("Loaded system info:", { systemId: this.systemId, systemName: this.systemName });
        }
      } catch (error) {
        console.error("Feil ved henting av system-informasjon:", error);
      } finally {
        this.isLoadingSystemInfo = false;
      }
    },
    
    async saveSystemName() {
      if (!this.systemName || !this.systemName.trim()) {
        return;
      }
      
      try {
        const response = await fetch(`${this.apiBaseUrl}/api/v1/system/name`, {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
            "Accept": "application/json"
          },
          body: JSON.stringify({
            name: this.systemName.trim()
          })
        });
        
        const data = await response.json();
        
        if (data.status === "success") {
          this.saveMessage = "Systemnavn lagret";
          this.saveStatus = "success";
        } else {
          this.saveMessage = "Kunne ikke lagre systemnavn";
          this.saveStatus = "error";
        }
      } catch (error) {
        console.error("Feil ved lagring av systemnavn:", error);
        this.saveMessage = "Feil ved kommunikasjon med serveren";
        this.saveStatus = "error";
      }
    },


    async saveConfig() {
      this.isSaving = true;
      this.saveMessage = "";
      this.saveStatus = "";
      
      try {
        const response = await fetch(`${this.apiBaseUrl}/api/v1/config/passage`, {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
            "Accept": "application/json"
          },
          body: JSON.stringify({
            debounce_ms: parseInt(this.debounceMs),
            min_sensors: parseInt(this.minSensors),
            timeout_ms: parseInt(this.timeoutMs)
          })
        });
        
        const data = await response.json();
        
        if (data.status === "success") {
          this.saveMessage = "Innstillingene ble lagret";
          this.saveStatus = "success";
          
          // Oppdater verdiene fra responsen for å sikre at de matcher server-verdiene
          if (data.config) {
            this.debounceMs = data.config.debounce_ms;
            this.minSensors = data.config.min_sensors;
            this.timeoutMs = data.config.timeout_ms;
          }
        } else {
          this.saveMessage = "Kunne ikke lagre innstillingene";
          this.saveStatus = "error";
        }
      } catch (error) {
        console.error("Feil ved lagring av passering-konfigurasjon:", error);
        this.saveMessage = "Feil ved kommunikasjon med serveren";
        this.saveStatus = "error";
      } finally {
        this.isSaving = false;
      }
    }
  }
};
</script>

<template>
  <div>
    <h1>Konfigurering</h1>
    
    <div class="config-container">
      <!-- System-informasjon seksjon -->
      <div class="config-section">
        <h2>System Identifikasjon</h2>
        
        <div class="form-group">
          <label>System ID:</label>
          <div class="system-id-display">
            <span class="system-id-value">{{ displaySystemId }}</span>
            <div class="system-id-note">
              Generert automatisk fra enhetens MAC-adresse. Dette ID-et brukes for å isolere systemet fra andre Timergate-systemer i nærheten.
            </div>
          </div>
        </div>
        
        <div class="form-group">
          <label for="system-name">Systemnavn:</label>
          <div class="system-name-input">
            <input 
              id="system-name"
              type="text" 
              v-model="displaySystemName"
              placeholder="F.eks. Timergate Nord" 
              class="text-input"
              :disabled="isLoadingSystemInfo"
            />
            <button 
              @click="saveSystemName" 
              class="save-name-button"
              :disabled="!systemName || !systemName.trim() || isLoadingSystemInfo"
            >
              Lagre navn
            </button>
          </div>
        </div>
        
        <div class="form-group">
          <label>Discovery-status:</label>
          <div class="discovery-status-display">
            <span class="discovery-indicator" :class="{ active: discoveryActive }">
              {{ discoveryActive ? 'Aktiv - leter etter nye målestolper' : 'Inaktiv' }}
            </span>
          </div>
        </div>
      </div>
      <div class="config-section">
        <h2>Passerings-deteksjon</h2>
        
        <div class="form-group">
          <label for="debounce-ms">Debounce-tid (millisekunder):</label>
          <input 
            id="debounce-ms" 
            type="number" 
            v-model="debounceMs" 
            min="500" 
            max="15000" 
            step="250"
          />
          <div class="description">
            Minimumstid mellom passeringer. Sensorbrudd i denne perioden etter en passering vil bli ignorert.
          </div>
        </div>
        
        <div class="form-group">
          <label for="min-sensors">Minimum antall sensorer:</label>
          <input 
            id="min-sensors" 
            type="number" 
            v-model="minSensors" 
            min="1" 
            max="7" 
            step="1"
          />
          <div class="description">
            Antall unike sensorer som må utløses for å registrere en passering.
          </div>
        </div>
        
        <div class="form-group">
          <label for="timeout-ms">Timeout for sekvens (millisekunder):</label>
          <input 
            id="timeout-ms" 
            type="number" 
            v-model="timeoutMs" 
            min="100" 
            max="1000" 
            step="25"
          />
          <div class="description">
            Hvor lenge systemet venter på nok sensorer før sekvensen tilbakestilles.
          </div>
        </div>
        
        <div class="form-actions">
          <button 
            @click="saveConfig" 
            :disabled="isSaving"
            class="save-button"
          >
            {{ isSaving ? 'Lagrer...' : 'Lagre innstillinger' }}
          </button>
          
          <div v-if="saveMessage" :class="['save-message', saveStatus]">
            {{ saveMessage }}
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<style scoped>
.config-container {
  display: flex;
  flex-direction: column;
  gap: 24px;
}

.config-section {
  background-color: white;
  border-radius: 8px;
  padding: 24px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.05);
}

.config-section h2 {
  margin-top: 0;
  margin-bottom: 24px;
  color: #555;
  font-size: 18px;
  border-bottom: 1px solid #eee;
  padding-bottom: 12px;
}

.form-group {
  margin-bottom: 20px;
}

.form-group label {
  display: block;
  font-weight: bold;
  margin-bottom: 8px;
  color: #333;
}

.form-group input {
  width: 100%;
  padding: 10px;
  border: 1px solid #ddd;
  border-radius: 4px;
  font-size: 16px;
}

.description {
  margin-top: 6px;
  font-size: 14px;
  color: #777;
  line-height: 1.4;
}

.form-actions {
  margin-top: 30px;
}

.save-button {
  background-color: #0078D7;
  color: white;
  border: none;
  border-radius: 4px;
  padding: 10px 16px;
  font-weight: bold;
  cursor: pointer;
}

.save-button:hover {
  background-color: #0063b1;
}

.save-button:disabled {
  background-color: #ccc;
  cursor: not-allowed;
}

.save-message {
  margin-top: 10px;
  padding: 8px;
  border-radius: 4px;
}

.save-message.success {
  background-color: #e6f7e6;
  color: #2e7d32;
  border: 1px solid #c8e6c9;
}

.save-message.error {
  background-color: #ffebee;
  color: #c62828;
  border: 1px solid #ffcdd2;
}


/* System informasjon styling */
.system-id-display {
  display: flex;
  flex-direction: column;
  gap: 8px;
}

.system-id-value {
  font-family: monospace;
  font-size: 18px;
  font-weight: bold;
  background-color: #f5f5f5;
  padding: 12px 16px;
  border-radius: 4px;
  border: 1px solid #ddd;
  color: #333;
  letter-spacing: 1px;
}

.system-id-note {
  font-size: 12px;
  color: #666;
  line-height: 1.4;
  font-style: italic;
}

.system-name-input {
  display: flex;
  gap: 12px;
  align-items: center;
}

.system-name-input .text-input {
  flex: 1;
}

.save-name-button {
  background-color: #0078D7;
  color: white;
  border: none;
  border-radius: 4px;
  padding: 10px 16px;
  font-weight: 500;
  cursor: pointer;
  white-space: nowrap;
}

.save-name-button:hover:not(:disabled) {
  background-color: #0063b1;
}

.save-name-button:disabled {
  background-color: #ccc;
  cursor: not-allowed;
}

.discovery-status-display {
  padding: 8px 0;
}

.discovery-indicator {
  padding: 8px 12px;
  border-radius: 6px;
  font-size: 14px;
  font-weight: 500;
  display: inline-block;
}

.discovery-indicator.active {
  background-color: #e8f5e9;
  color: #2e7d32;
  border: 1px solid #c8e6c9;
}

.discovery-indicator:not(.active) {
  background-color: #ffebee;
  color: #c62828;
  border: 1px solid #ffcdd2;
}

/* Responsive adjustments for system config */
@media (max-width: 768px) {
  .system-name-input {
    flex-direction: column;
    align-items: stretch;
  }
  
  .save-name-button {
    width: 100%;
  }
}




</style>