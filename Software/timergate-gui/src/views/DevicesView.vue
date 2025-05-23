<template>
  <div>
    <h1>Enheter</h1>
    
    <div class="devices-container">
      <div class="status-summary">
        <div class="status-card">
          <div class="status-value">{{ poles.length }}</div>
          <div class="status-label">Tilkoblede enheter</div>
        </div>
        <div class="status-card">
          <div class="status-value">{{ countActiveSensors() }}</div>
          <div class="status-label">Aktive sensorer</div>
        </div>
        <button @click="scanForDevices" class="scan-button">
          <span class="icon">🔍</span>
          Søk etter enheter
        </button>
      </div>
      
      <div v-if="poles.length > 0" class="devices-list">
        <div v-for="pole in poles" :key="pole.id" class="device-card">
          <div class="device-header">
            <div class="device-title">
              <h2>{{ pole.name }}</h2>
              <div class="device-status" :class="getStatusClass(pole)">
                {{ getConnectionStatus(pole) }}
              </div>
            </div>
            <div class="device-actions-compact">
              <button @click="expandDevice(pole)" class="icon-button" v-if="!isExpanded(pole)">
                <span class="icon">⬇️</span>
              </button>
              <button @click="collapseDevice(pole)" class="icon-button" v-else>
                <span class="icon">⬆️</span>
              </button>
            </div>
          </div>
          
          <div class="device-info">
            <div class="info-row">
              <div class="info-label">MAC-adresse:</div>
              <div class="info-value">{{ pole.mac }}</div>
            </div>
            <div class="info-row">
              <div class="info-label">Aktive sensorer:</div>
              <div class="info-value">{{ countPoleActiveSensors(pole) }} / {{ getEnabledSensors(pole).length }}</div>
            </div>
            <div class="info-row">
              <div class="info-label">Batteristatus:</div>
              <div class="info-value">{{ pole.battery || 'N/A' }}</div>
            </div>
            <div class="info-row">
              <div class="info-label">Sist aktivitet:</div>
              <div class="info-value">{{ getLastActivity(pole) }}</div>
            </div>
          </div>
          
          <div v-if="isExpanded(pole)" class="device-expanded">
            <div class="sensor-section">
              <h3>Sensorstatus</h3>
              <div class="sensor-grid">
                <div v-for="(value, index) in pole.values" :key="index" class="sensor-item">
                  <div class="sensor-label">Sensor {{ index }}</div>
                  <div class="sensor-value" :class="{ 'sensor-active': isSensorActive(value), 'sensor-inactive': !isSensorActive(value) }">
                    {{ value || 0 }}
                  </div>
                  <div class="sensor-bar-container">
                    <div class="sensor-bar" :style="{ width: getSensorBarWidth(value) }"></div>
                    <div class="sensor-threshold" :style="{ left: getSensorThresholdPosition(pole, index) }"></div>
                  </div>
                </div>
              </div>
            </div>
            
            <div class="action-section">
              <h3>Handlinger</h3>
              <div class="action-buttons">
                <button @click="openRestartModal(pole)" class="action-button restart">
                  <span class="icon">🔄</span>
                  Restart enhet
                </button>
                <button @click="calibrateSensors(pole)" class="action-button calibrate">
                  <span class="icon">⚙️</span>
                  Kalibrer sensorer
                </button>
                <button @click="identifyDevice(pole)" class="action-button identify">
                  <span class="icon">💡</span>
                  Finn enhet
                </button>
                <button @click="openRenameModal(pole)" class="action-button rename">
                  <span class="icon">✏️</span>
                  Endre navn
                </button>
                <button @click="openPowerModal(pole)" class="action-button power" :class="{ 'power-off': !isPowerOn(pole) }">
                  <span class="icon">{{ isPowerOn(pole) ? '🔌' : '💤' }}</span>
                  {{ isPowerOn(pole) ? 'Slå av enhet' : 'Slå på enhet' }}
                </button>
              </div>
            </div>
            
            <div class="diagnostics-section">
              <h3>Diagnostikk</h3>
              <div class="action-buttons">
                <button @click="checkDiagnostics(pole)" class="action-button diagnostics">
                  <span class="icon">🩺</span>
                  Kjør diagnostikk
                </button>
                <button @click="viewDeviceLog(pole)" class="action-button logs">
                  <span class="icon">📋</span>
                  Vis logg
                </button>
              </div>
            </div>
          </div>
          
          <div class="device-actions-footer" v-if="!isExpanded(pole)">
            <button @click="expandDevice(pole)" class="expand-button">
              Vis detaljer
            </button>
          </div>
        </div>
      </div>
      
      <div v-else class="no-devices">
        <div class="empty-state">
          <div class="empty-icon">📡</div>
          <h3>Ingen enheter tilkoblet</h3>
          <p>Søk etter enheter eller koble til en ny målestolpe for å komme i gang.</p>
          <button @click="scanForDevices" class="scan-button">Søk etter enheter</button>
        </div>
      </div>
    </div>
    
    <!-- Restart Modal -->
    <div v-if="showRestartModal" class="modal-overlay">
      <div class="modal-content">
        <div class="modal-header">
          <h2>Restart enhet</h2>
          <button @click="cancelRestart" class="close-button">×</button>
        </div>
        
        <div class="modal-body">
          <p>Velg type restart for <strong>{{ selectedPole ? selectedPole.name : '' }}</strong> ({{ selectedPole ? selectedPole.mac : '' }})</p>
          
          <div class="restart-options">
            <div class="restart-option">
              <input type="radio" id="restart-soft" name="restart-type" value="1" v-model="restartType">
              <label for="restart-soft">
                <strong>Myk restart</strong>
                <p>Standard restart av enheten. Beholder alle innstillinger.</p>
              </label>
            </div>
            
            <div class="restart-option">
              <input type="radio" id="restart-hard" name="restart-type" value="2" v-model="restartType">
              <label for="restart-hard">
                <strong>Hard restart</strong>
                <p>Tvungen restart. Bruk hvis enheten ikke responderer normalt.</p>
              </label>
            </div>
            
            <div class="restart-option">
              <input type="radio" id="restart-factory" name="restart-type" value="3" v-model="restartType">
              <label for="restart-factory">
                <strong>Factory reset</strong>
                <p>Tilbakestill enheten til fabrikkinnstillinger. Alle konfigurasjoner vil gå tapt.</p>
              </label>
            </div>
          </div>
          
          <div class="warning-message" v-if="restartType === '3'">
            <strong>Advarsel:</strong> Factory reset vil slette alle lagrede innstillinger og kalibreringer. 
            Enheten må konfigureres på nytt etter restart.
          </div>
        </div>
        
        <div class="modal-footer">
          <button @click="cancelRestart" class="cancel-button">Avbryt</button>
          <button @click="confirmRestart" class="confirm-button restart" :disabled="isRestarting">
            {{ isRestarting ? 'Restarter...' : 'Restart' }}
          </button>
        </div>
      </div>
    </div>
    
    <!-- Rename Modal -->
    <div v-if="showRenameModal" class="modal-overlay">
      <div class="modal-content">
        <div class="modal-header">
          <h2>Endre navn</h2>
          <button @click="cancelRename" class="close-button">×</button>
        </div>
        
        <div class="modal-body">
          <p>Angi nytt navn for enheten ({{ selectedPole ? selectedPole.mac : '' }})</p>
          
          <div class="form-group">
            <label for="device-name">Navn:</label>
            <input 
              type="text" 
              id="device-name" 
              v-model="newDeviceName" 
              placeholder="F.eks. Målestolpe Start"
              class="text-input"
            >
          </div>
        </div>
        
        <div class="modal-footer">
          <button @click="cancelRename" class="cancel-button">Avbryt</button>
          <button @click="confirmRename" class="confirm-button" :disabled="!newDeviceName">
            Lagre
          </button>
        </div>
      </div>
    </div>
    
    <!-- Logs Modal -->
    <div v-if="showLogsModal" class="modal-overlay">
      <div class="modal-content logs-modal">
        <div class="modal-header">
          <h2>Enhetslogg: {{ selectedPole ? selectedPole.name : '' }}</h2>
          <button @click="closeLogsModal" class="close-button">×</button>
        </div>
        
        <div class="modal-body">
          <div class="log-filters">
            <select v-model="logLevel" class="select-input">
              <option value="all">Alle nivåer</option>
              <option value="info">Info</option>
              <option value="warning">Advarsel</option>
              <option value="error">Feil</option>
            </select>
            <button @click="refreshLogs" class="refresh-button">
              <span class="icon">🔄</span> Oppdater
            </button>
          </div>
          
          <div class="log-entries">
            <div v-if="deviceLogs.length === 0" class="empty-logs">
              Ingen logginnslag funnet
            </div>
            <div v-for="(log, index) in filteredLogs" :key="index" class="log-entry" :class="getLogClass(log)">
              <div class="log-time">{{ formatLogTime(log.time) }}</div>
              <div class="log-level">{{ log.level }}</div>
              <div class="log-message">{{ log.message }}</div>
            </div>
          </div>
        </div>
        
        <div class="modal-footer">
          <button @click="closeLogsModal" class="cancel-button">Lukk</button>
          <button @click="downloadLogs" class="confirm-button">
            <span class="icon">💾</span> Last ned logg
          </button>
        </div>
      </div>
    </div>


    <!-- Power Off Modal -->
    <div v-if="showPowerModal" class="modal-overlay">
      <div class="modal-content">
        <div class="modal-header">
          <h2>Slå av enhet</h2>
          <button @click="cancelPowerOff" class="close-button">×</button>
        </div>
        
        <div class="modal-body">
          <div class="warning-message">
            <strong>⚠️ Advarsel:</strong> Dette vil sette enheten i deep sleep-modus.
          </div>
          
          <p>Er du sikker på at du vil slå av <strong>{{ selectedPole ? selectedPole.name : '' }}</strong>?</p>
          
          <div class="power-off-info">
            <h4>Viktig informasjon:</h4>
            <ul>
              <li>Enheten vil gå i deep sleep og bruke minimal strøm</li>
              <li>Enheten vil <strong>ikke</strong> motta nettverkskommandoer</li>
              <li>Du må <strong>fysisk trykke reset-knappen</strong> på enheten for å slå den på igjen</li>
              <li>All nettverkstilkobling vil bli brutt</li>
            </ul>
          </div>
        </div>
        
        <div class="modal-footer">
          <button @click="cancelPowerOff" class="cancel-button">Avbryt</button>
          <button @click="confirmPowerOff" class="confirm-button power-off" :disabled="isPoweringOff">
            {{ isPoweringOff ? 'Slår av...' : 'Slå av enhet' }}
          </button>
        </div>
      </div>
    </div>



    
    <!-- Toast notifications -->
    <div class="toast-container">
      <div v-for="(toast, index) in toasts" :key="index" class="toast" :class="toast.type">
        <div class="toast-content">
          <span class="toast-icon">{{ getToastIcon(toast.type) }}</span>
          <span class="toast-message">{{ toast.message }}</span>
        </div>
        <button @click="dismissToast(index)" class="toast-close">×</button>
      </div>
    </div>
  </div>
</template>

<script>
export default {
  props: {
    poles: {
      type: Array,
      default: () => []
    },
    serverAddress: {
      type: String,
      default: "timergate.local"
    }
  },
  data() {
    return {
      // Expanded state
      expandedPoles: {},

      // Power modal
      showPowerModal: false,
      isPoweringOff: false,
      
      // Restart modal
      showRestartModal: false,
      selectedPole: null,
      restartType: "1", // Standard: myk restart
      isRestarting: false,
      
      // Rename modal
      showRenameModal: false,
      newDeviceName: "",

      powerStatus: {}, // Holder styr på strømstatus for hver enhet (indeksert etter MAC)
      
      // Logs modal
      showLogsModal: false,
      deviceLogs: [],
      logLevel: "all",
      
      // Toast notifications
      toasts: [],
      
      // Status flagg
      isScanning: false,
      isCalibratingMap: {},
      isDiagnosticsRunning: false,

    };
  },
  computed: {
    filteredLogs() {
      if (this.logLevel === "all") {
        return this.deviceLogs;
      }
      return this.deviceLogs.filter(log => log.level.toLowerCase() === this.logLevel);
    }
  },
  methods: {
    // Hjelpefunksjoner

    isPowerOn(pole) {
      // Sjekk om vi har en registrert status for denne enheten
      if (pole.mac in this.powerStatus) {
        return this.powerStatus[pole.mac];
      }
      // Anta at enheten er på hvis det ikke er en eksplisitt status
      return true;
    },

    isExpanded(pole) {
      return !!this.expandedPoles[pole.mac];
    },
    
    expandDevice(pole) {
      this.expandedPoles = { ...this.expandedPoles, [pole.mac]: true };
        },
      collapseDevice(pole) {
        this.expandedPoles = { ...this.expandedPoles, [pole.mac]: false };
      },
    
    countActiveSensors() {
      let count = 0;
      this.poles.forEach(pole => {
        count += this.countPoleActiveSensors(pole);
      });
      return count;
    },
    
    countPoleActiveSensors(pole) {
      if (!pole.values) return 0;
      return pole.values.filter(v => this.isSensorActive(v)).length;
    },
    
    isSensorActive(value) {
      return value && value > 0;
    },
    
    getSensorBarWidth(value) {
      if (!value) return '0%';
      // Normaliser verdien til en prosent (antar maksverdi er 4096 for ADC)
      const percent = Math.min(100, Math.max(0, (value / 4096) * 100));
      return `${percent}%`;
    },
    
    getSensorThresholdPosition(pole, index) {
      if (!pole.br_limit || !pole.br_limit[index]) return '50%';
      // Beregn posisjon for break-limit i prosent
      const percent = Math.min(100, Math.max(0, (pole.br_limit[index] / 4096) * 100));
      return `${percent}%`;
    },
    
    getEnabledSensors(pole) {
      if (!pole.enabled) return [];
      return pole.enabled.filter(e => e === true);
    },
    
    getConnectionStatus(pole) {
      return pole.values ? "Tilkoblet" : "Frakoblet";
    },
    
    getStatusClass(pole) {
      return pole.values ? "connected" : "disconnected";
    },
    
    getLastActivity(pole) {
      // Dette er en placeholder - i en faktisk implementasjon ville dette hentes fra faktiske data
      return "Nettopp";
    },
    
    formatLogTime(timestamp) {
      if (!timestamp) return "";
      const date = new Date(timestamp);
      return date.toLocaleTimeString();
    },
    
    getLogClass(log) {
      return {
        'log-info': log.level === 'INFO',
        'log-warning': log.level === 'WARNING',
        'log-error': log.level === 'ERROR'
      };
    },
    
    getToastIcon(type) {
      switch(type) {
        case 'success': return '✅';
        case 'error': return '❌';
        case 'warning': return '⚠️';
        case 'info': return 'ℹ️';
        default: return 'ℹ️';
      }
    },
    
    // Actions
    async scanForDevices() {
      if (this.isScanning) return;
      
      this.isScanning = true;
      this.showToast('info', 'Søker etter enheter...');
      
      try {
        // Dette er en placeholder - i en faktisk implementasjon ville dette kalle et API
        await new Promise(resolve => setTimeout(resolve, 1500));
        
        this.showToast('success', 'Søk fullført');
      } catch (error) {
        this.showToast('error', 'Kunne ikke søke etter enheter');
        console.error('Feil ved søk etter enheter:', error);
      } finally {
        this.isScanning = false;
      }
    },
    
    // Restart Modal
    openRestartModal(pole) {
      this.selectedPole = pole;
      this.showRestartModal = true;
      this.restartType = "1"; // Reset til standard
    },
    
    cancelRestart() {
      this.showRestartModal = false;
      this.selectedPole = null;
    },
    
    async confirmRestart() {
      if (!this.selectedPole) return;
      
      this.isRestarting = true;
      
      try {
        const response = await fetch(`http://${this.serverAddress}/api/v1/pole/restart`, {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
            "Accept": "application/json"
          },
          body: JSON.stringify({
            mac: this.selectedPole.mac,
            type: parseInt(this.restartType)
          })
        });
        
        const data = await response.json();
        
        if (data.status === "success") {
          // Vis toast melding
          let message = '';
          switch(this.restartType) {
            case '1': message = 'Myk restart startet'; break;
            case '2': message = 'Hard restart startet'; break;
            case '3': message = 'Factory reset startet'; break;
            default: message = 'Restart startet'; break;
          }
          
          this.showToast('success', `${message} for ${this.selectedPole.name}`);
          
          // Lukk modal
          this.showRestartModal = false;
          this.selectedPole = null;
        } else {
          this.showToast('error', `Kunne ikke restarte enheten: ${data.message || 'Ukjent feil'}`);
        }
      } catch (error) {
        console.error("Feil ved restart av enhet:", error);
        this.showToast('error', 'Feil ved kommunikasjon med serveren');
      } finally {
        this.isRestarting = false;
      }
    },


    // Power Modal
    openPowerModal(pole) {
      const currentStatus = this.isPowerOn(pole);
      
      if (currentStatus) {
        // Enheten er på, vis warning for power off
        this.selectedPole = pole;
        this.showPowerModal = true;
      } else {
        // Enheten er av, vis informasjon om reset
        this.showToast('info', `${pole.name} er i deep sleep. Trykk reset-knappen på enheten for å slå den på igjen.`);
      }
    },

    cancelPowerOff() {
      this.showPowerModal = false;
      this.selectedPole = null;
    },

    async confirmPowerOff() {
      if (!this.selectedPole) return;
      
      this.isPoweringOff = true;
      
      try {
        this.showToast('info', `Slår av ${this.selectedPole.name}...`);
        
        const response = await fetch(`http://${this.serverAddress}/api/v1/pole/power`, {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
            "Accept": "application/json"
          },
          body: JSON.stringify({
            mac: this.selectedPole.mac,
            power: "off"
          })
        });
        
        const data = await response.json();
        
        if (data.status === "success") {
          // Oppdater strømstatusen i lokal tilstand
          this.powerStatus = {
            ...this.powerStatus,
            [this.selectedPole.mac]: false
          };
          
          this.showToast('success', `${this.selectedPole.name} går nå i deep sleep. Trykk reset-knappen på enheten for å slå den på igjen.`);
          
          // Lukk modal
          this.showPowerModal = false;
          this.selectedPole = null;
        } else {
          this.showToast('error', `Kunne ikke slå av ${this.selectedPole.name}: ${data.message || 'Ukjent feil'}`);
        }
      } catch (error) {
        console.error("Feil ved strømkontroll:", error);
        this.showToast('error', 'Feil ved kommunikasjon med serveren');
      } finally {
        this.isPoweringOff = false;
      }
    },







    
    // async togglePower(pole) {
    //   try {
    //     const currentStatus = this.isPowerOn(pole);
    //     const newStatus = !currentStatus;
        
    //     // Spesiell håndtering for "slå på" kommando
    //     if (newStatus === true) {
    //       // Enheten er av (i deep sleep) og kan ikke motta kommandoer
    //       this.showToast('info', `${pole.name} er i deep sleep. Trykk reset-knappen på enheten for å slå den på igjen.`);
    //       return; // Ikke send API-kall
    //     }
        
    //     // Vis at handlingen er i gang (kun for "slå av")
    //     this.showToast('info', `Slår av ${pole.name}...`);
        
    //     const response = await fetch(`http://${this.serverAddress}/api/v1/pole/power`, {
    //       method: "POST",
    //       headers: {
    //         "Content-Type": "application/json",
    //         "Accept": "application/json"
    //       },
    //       body: JSON.stringify({
    //         mac: pole.mac,
    //         power: "off"
    //       })
    //     });
        
    //     const data = await response.json();
        
    //     if (data.status === "success") {
    //       // Oppdater strømstatusen i lokal tilstand
    //       this.powerStatus = {
    //         ...this.powerStatus,
    //         [pole.mac]: false
    //       };
          
    //       this.showToast('success', `${pole.name} går nå i deep sleep. Trykk reset-knappen på enheten for å slå den på igjen.`);
    //     } else {
    //       this.showToast('error', `Kunne ikke slå av ${pole.name}: ${data.message || 'Ukjent feil'}`);
    //     }
    //   } catch (error) {
    //     console.error("Feil ved strømkontroll:", error);
    //     this.showToast('error', 'Feil ved kommunikasjon med serveren');
    //   }
    // },


    // async togglePower(pole) {
    //   try {
    //     const currentStatus = this.isPowerOn(pole);
    //     const newStatus = !currentStatus;
        
    //     // Vis at handlingen er i gang
    //     this.showToast('info', `${newStatus ? 'Slår på' : 'Slår av'} ${pole.name}...`);
        
    //       const response = await fetch(`http://${this.serverAddress}/api/v1/pole/power`, {
    //       method: "POST",
    //       headers: {
    //         "Content-Type": "application/json",
    //         "Accept": "application/json"
    //       },
    //       body: JSON.stringify({
    //         mac: pole.mac,
    //         power: newStatus ? "on" : "off"
    //       })
    //     });
        
    //     const data = await response.json();
        
    //     if (data.status === "success") {
    //       // Oppdater strømstatusen i lokal tilstand
    //       this.powerStatus = {
    //         ...this.powerStatus,
    //         [pole.mac]: newStatus
    //       };
          
    //       this.showToast('success', `${pole.name} er nå ${newStatus ? 'slått på' : 'slått av'}`);
    //     } else {
    //       this.showToast('error', `Kunne ikke ${newStatus ? 'slå på' : 'slå av'} ${pole.name}: ${data.message || 'Ukjent feil'}`);
    //     }
    //   } catch (error) {
    //     console.error("Feil ved strømkontroll:", error);
    //     this.showToast('error', 'Feil ved kommunikasjon med serveren');
    //   }
    // },




    // Rename Modal
    openRenameModal(pole) {
      this.selectedPole = pole;
      this.newDeviceName = pole.name;
      this.showRenameModal = true;
    },
    
    cancelRename() {
      this.showRenameModal = false;
      this.selectedPole = null;
      this.newDeviceName = "";
    },
    
    confirmRename() {
      if (!this.selectedPole || !this.newDeviceName.trim()) return;
      
      // Siden vi ikke har et faktisk API-endepunkt for dette ennå, bare oppdater lokalt
      this.selectedPole.name = this.newDeviceName.trim();
      this.showToast('success', `Enhetsnavn endret til ${this.newDeviceName}`);
      
      this.showRenameModal = false;
      this.selectedPole = null;
      this.newDeviceName = "";
    },
    
    // Kalibrering
    async calibrateSensors(pole) {
      // Sett kalibreringsflagg
      //this.$set(this.isCalibratingMap, pole.mac, true);
      this.isCalibratingMap = { ...this.isCalibratingMap, [pole.mac]: true };
      
      try {
        const response = await fetch(`http://${this.serverAddress}/api/v1/time/hsearch`, {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
            "Accept": "application/json"
          },
          body: JSON.stringify({
            channel: 0,
            mac: pole.mac // Legg til MAC-adresse for spesifikk kalibrering
          })
        });
        
        const data = await response.json();
        
        if (data.status === "success") {
          this.showToast('success', `Kalibrering startet for ${pole.name}`);
        } else {
          this.showToast('error', `Kunne ikke starte kalibrering: ${data.message || 'Ukjent feil'}`);
        }
      } catch (error) {
        console.error("Feil ved kalibrering:", error);
        this.showToast('error', 'Feil ved kommunikasjon med serveren');
      } finally {
        // Fjern kalibreringsflagg etter en forsinkelse for å indikere at prosessen er i gang
        setTimeout(() => {
          //this.$set(this.isCalibratingMap, pole.mac, false);
          this.isCalibratingMap = { ...this.isCalibratingMap, [pole.mac]: false };
        }, 2000);
      }
    },
    
    // Finn enhet
    async identifyDevice(pole) {
      try {
        // Dette er en placeholder - i en faktisk implementasjon ville dette kalle et API
        this.showToast('info', `Sender blinksignal til ${pole.name}...`);
        
        // Simuler API-kall
        await new Promise(resolve => setTimeout(resolve, 1000));
        
        this.showToast('success', `${pole.name} blinker nå for identifikasjon`);
      } catch (error) {
        console.error("Feil ved identifisering av enhet:", error);
        this.showToast('error', 'Feil ved kommunikasjon med serveren');
      }
    },
    
    // Kjør diagnostikk
    async checkDiagnostics(pole) {
      if (this.isDiagnosticsRunning) return;
      
      this.isDiagnosticsRunning = true;
      this.showToast('info', `Kjører diagnostikk på ${pole.name}...`);
      
      try {
        // Dette er en placeholder - i en faktisk implementasjon ville dette kalle et API
        await new Promise(resolve => setTimeout(resolve, 2000));
        
        this.showToast('success', `Diagnostikk fullført for ${pole.name}: Alle systemer fungerer normalt`);
      } catch (error) {
        console.error("Feil ved kjøring av diagnostikk:", error);
        this.showToast('error', 'Feil ved kommunikasjon med serveren');
      } finally {
        this.isDiagnosticsRunning = false;
      }
    },
    
    // Logs
    viewDeviceLog(pole) {
      this.selectedPole = pole;
      this.deviceLogs = this.generateDummyLogs(); // Dummy-data for eksempelvisning
      this.showLogsModal = true;
    },
    
    closeLogsModal() {
      this.showLogsModal = false;
      this.selectedPole = null;
      this.deviceLogs = [];
    },
    
    refreshLogs() {
      this.deviceLogs = this.generateDummyLogs(); // Oppdater med nye dummy-data
      this.showToast('info', 'Loggen er oppdatert');
    },
    
    downloadLogs() {
      // Dette er en placeholder - i en faktisk implementasjon ville dette laste ned en loggfil
      this.showToast('success', 'Logger lastet ned');
    },
    
    // Genererer dummy-loggdata for eksempelvisning
    generateDummyLogs() {
      const now = Date.now();
      const logs = [];
      
      // Legg til noen eksempel-logger
      logs.push({ time: now, level: 'INFO', message: 'System startet opp' });
      logs.push({ time: now - 60000, level: 'INFO', message: 'Wi-Fi-tilkobling opprettet' });
      logs.push({ time: now - 120000, level: 'WARNING', message: 'Sensor 2 viser ustabile verdier' });
      logs.push({ time: now - 180000, level: 'INFO', message: 'Kalibrering fullført' });
      logs.push({ time: now - 240000, level: 'ERROR', message: 'Kunne ikke koble til AP' });
      logs.push({ time: now - 300000, level: 'INFO', message: 'Restart gjennomført' });
      
      // Sorter etter tid (nyeste først)
      return logs.sort((a, b) => b.time - a.time);
    },
    
    // Toast notifications
    showToast(type, message, timeout = 3000) {
      const toast = { type, message };
      this.toasts.push(toast);
      
      // Automatisk fjerning etter timeout
      setTimeout(() => {
        const index = this.toasts.indexOf(toast);
        if (index !== -1) {
          this.toasts.splice(index, 1);
        }
      }, timeout);
    },
    
    dismissToast(index) {
      this.toasts.splice(index, 1);
    },
    mounted() {
      // Initialiser strømstatus for alle målestolper
      this.poles.forEach(pole => {
        if (!(pole.mac in this.powerStatus)) {
          this.powerStatus[pole.mac] = true; // Anta at alle er på ved oppstart
        }
      });
    },
    watch: {
      poles: {
        handler(newPoles) {
          // Oppdater strømstatus for nye målestolper
          newPoles.forEach(pole => {
            if (!(pole.mac in this.powerStatus)) {
              this.powerStatus[pole.mac] = true; // Anta at nye enheter er på
            }
          });
        },
        deep: true,
        immediate: true
      }
    }
  }
};
</script>

<style scoped>
/* Base styles */
h1 {
  margin-bottom: 24px;
  color: #333;
}

h2, h3 {
  margin-top: 0;
  color: #333;
}

/* Status summary */
.status-summary {
  display: flex;
  flex-wrap: wrap;
  gap: 16px;
  margin-bottom: 24px;
}

.status-card {
  background-color: white;
  border-radius: 8px;
  padding: 16px 24px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
  flex: 1;
  min-width: 120px;
  text-align: center;
}

.status-value {
  font-size: 32px;
  font-weight: bold;
  color: #0078D7;
  margin-bottom: 4px;
}

.status-label {
  color: #666;
  font-size: 14px;
}

.scan-button {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 8px;
  background-color: #0078D7;
  color: white;
  border: none;
  border-radius: 8px;
  padding: 0 24px;
  font-weight: 500;
  cursor: pointer;
  transition: background-color 0.2s;
}

.scan-button:hover {
  background-color: #0063b1;
}

.scan-button:disabled {
  background-color: #ccc;
  cursor: not-allowed;
}

/* Devices list */
.devices-list {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(400px, 1fr));
  gap: 20px;
}

.device-card {
  background-color: white;
  border-radius: 8px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
  padding: 20px;
  transition: box-shadow 0.3s;
}

.device-card:hover {
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.15);
}

/* Device header */
.device-header {
  display: flex;
  justify-content: space-between;
  align-items: flex-start;
  margin-bottom: 16px;
  padding-bottom: 16px;
  border-bottom: 1px solid #eee;
}

.device-title {
  display: flex;
  flex-direction: column;
  gap: 8px;
}

.device-title h2 {
  margin: 0;
  font-size: 18px;
}

.device-status {
  display: inline-block;
  padding: 4px 8px;
  border-radius: 4px;
  font-size: 12px;
  font-weight: bold;
}

.device-status.connected {
  background-color: #e8f5e9;
  color: #2e7d32;
}

.device-status.disconnected {
  background-color: #ffebee;
  color: #c62828;
}

/* Device info */
.device-info {
  margin-bottom: 20px;
}

.info-row {
  display: flex;
  margin-bottom: 8px;
}

.info-label {
  width: 120px;
  color: #666;
}

.info-value {
  font-weight: 500;
  color: #333;
}

/* Device actions footer */
.device-actions-footer {
  display: flex;
  justify-content: center;
  margin-top: 16px;
}

.expand-button {
  background: none;
  border: 1px solid #ddd;
  border-radius: 4px;
  padding: 8px 16px;
  color: #666;
  cursor: pointer;
  transition: all 0.2s;
}

.expand-button:hover {
  background-color: #f5f5f5;
  border-color: #ccc;
}

/* Icons */
.icon {
  font-size: 1.2em;
}

.icon-button {
  background: none;
  border: none;
  cursor: pointer;
  font-size: 18px;
  padding: 4px;
  border-radius: 4px;
  transition: background-color 0.2s;
}

.icon-button:hover {
  background-color: #f5f5f5;
}

/* Expanded device sections */
.device-expanded {
  margin-top: 20px;
  display: flex;
  flex-direction: column;
  gap: 24px;
}

.sensor-section, .action-section, .diagnostics-section {
  background-color: #f9f9f9;
  border-radius: 8px;
  padding: 16px;
  border: 1px solid #eee;
}

.sensor-section h3, .action-section h3, .diagnostics-section h3 {
  margin-top: 0;
  margin-bottom: 16px;
  font-size: 16px;
  color: #555;
}

/* Sensor grid */
.sensor-grid {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(140px, 1fr));
  gap: 12px;
}

.sensor-item {
  background-color: white;
  border-radius: 6px;
  padding: 12px;
  box-shadow: 0 1px 3px rgba(0, 0, 0, 0.05);
}

.sensor-label {
  font-size: 13px;
  color: #666;
  margin-bottom: 4px;
}

.sensor-value {
  font-weight: bold;
  margin-bottom: 8px;
}

.sensor-active {
  color: #2e7d32;
}

.sensor-inactive {
  color: #9e9e9e;
}

.sensor-bar-container {
  height: 8px;
  background-color: #eee;
  border-radius: 4px;
  position: relative;
  overflow: hidden;
}

.sensor-bar {
  height: 100%;
  background-color: #4caf50;
  border-radius: 4px;
  transition: width 0.5s;
}

.sensor-threshold {
  position: absolute;
  top: 0;
  width: 2px;
  height: 100%;
  background-color: #f44336;
}

/* Action buttons */
.action-buttons {
  display: flex;
  flex-wrap: wrap;
  gap: 12px;
}

.action-button {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 10px 16px;
  border: none;
  border-radius: 6px;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.2s;
}

.action-button.restart {
  background-color: #ffebee;
  color: #c62828;
}

.action-button.restart:hover {
  background-color: #ffcdd2;
}

.action-button.calibrate {
  background-color: #e8f5e9;
  color: #2e7d32;
}

.action-button.calibrate:hover {
  background-color: #c8e6c9;
}

.action-button.identify {
  background-color: #e3f2fd;
  color: #1565c0;
}

.action-button.identify:hover {
  background-color: #bbdefb;
}

.action-button.rename {
  background-color: #fff3e0;
  color: #e65100;
}

.action-button.power {
  background-color: #ffebee;
  color: #c62828;
}

.action-button.power:hover {
  background-color: #f89ba4;
}

.action-button.power-off {
  background-color: #e0e0e0;
  color: #616161;
}

.action-button.power-off:hover {
  background-color: #bdbdbd;
}


.action-button.rename:hover {
  background-color: #ffe0b2;
}

.action-button.diagnostics {
  background-color: #f3e5f5;
  color: #7b1fa2;
}

.action-button.diagnostics:hover {
  background-color: #e1bee7;
}

.action-button.logs {
  background-color: #efebe9;
  color: #4e342e;
}

.action-button.logs:hover {
  background-color: #d7ccc8;
}

/* No devices */
.no-devices {
  width: 100%;
}

.empty-state {
  background-color: white;
  border-radius: 8px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
  padding: 48px 24px;
  text-align: center;
}

.empty-icon {
  font-size: 48px;
  margin-bottom: 16px;
}

.empty-state h3 {
  margin-top: 0;
  margin-bottom: 8px;
  font-size: 20px;
}

.empty-state p {
  color: #666;
  margin-bottom: 24px;
  max-width: 400px;
  margin-left: auto;
  margin-right: auto;
}

/* Modal styles */
.modal-overlay {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background-color: rgba(0, 0, 0, 0.6);
  display: flex;
  justify-content: center;
  align-items: center;
  z-index: 1000;
}

.modal-content {
  background-color: white;
  border-radius: 8px;
  width: 500px;
  max-width: 90vw;
  max-height: 85vh;
  box-shadow: 0 4px 20px rgba(0, 0, 0, 0.15);
  display: flex;
  flex-direction: column;
}

.logs-modal {
  width: 700px;
  height: 600px;
}

.modal-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 16px 24px;
  border-bottom: 1px solid #eee;
}

.modal-header h2 {
  margin: 0;
  font-size: 20px;
}

.close-button {
  background: none;
  border: none;
  font-size: 24px;
  cursor: pointer;
  color: #666;
}

.modal-body {
  padding: 24px;
  flex: 1;
  overflow-y: auto;
}

.modal-footer {
  display: flex;
  justify-content: flex-end;
  gap: 12px;
  padding: 16px 24px;
  border-top: 1px solid #eee;
}

/* Form elements */
.form-group {
  margin-bottom: 16px;
}

.form-group label {
  display: block;
  margin-bottom: 8px;
  font-weight: 500;
}

.text-input {
  width: 100%;
  padding: 10px;
  border: 1px solid #ddd;
  border-radius: 4px;
  font-size: 16px;
}

.select-input {
  padding: 8px 12px;
  border: 1px solid #ddd;
  border-radius: 4px;
  font-size: 14px;
}

/* Restart options */
.restart-options {
  margin: 24px 0;
}

.restart-option {
  display: flex;
  margin-bottom: 16px;
  align-items: flex-start;
}

.restart-option input {
  margin-top: 4px;
  margin-right: 12px;
}

.restart-option label {
  flex: 1;
}

.restart-option label strong {
  display: block;
  margin-bottom: 4px;
  color: #333;
}

.restart-option label p {
  margin: 0;
  font-size: 14px;
  color: #666;
}

.warning-message {
  background-color: #fff8e1;
  border-left: 4px solid #ffc107;
  padding: 12px 16px;
  margin-top: 16px;
  color: #bf360c;
  font-size: 14px;
}

/* Buttons */
.cancel-button {
  padding: 10px 16px;
  background-color: #f5f5f5;
  border: 1px solid #ddd;
  border-radius: 4px;
  font-weight: 500;
  cursor: pointer;
}

.cancel-button:hover {
  background-color: #e0e0e0;
}

.confirm-button {
  padding: 10px 16px;
  background-color: #0078D7;
  color: white;
  border: none;
  border-radius: 4px;
  font-weight: 500;
  cursor: pointer;
}

.confirm-button:hover:not(:disabled) {
  background-color: #0063b1;
}

.confirm-button:disabled {
  background-color: #ccc;
  cursor: not-allowed;
}

.confirm-button.restart {
  background-color: #c62828;
}

.confirm-button.restart:hover:not(:disabled) {
  background-color: #b71c1c;
}

.refresh-button {
  background: none;
  border: 1px solid #ddd;
  border-radius: 4px;
  padding: 6px 12px;
  display: flex;
  align-items: center;
  gap: 6px;
  cursor: pointer;
}

.refresh-button:hover {
  background-color: #f5f5f5;
}

/* Logs */
.log-filters {
  display: flex;
  justify-content: space-between;
  margin-bottom: 16px;
}

.log-entries {
  background-color: #f5f5f5;
  border-radius: 4px;
  overflow-y: auto;
  max-height: 350px;
  font-family: monospace;
}

.log-entry {
  display: flex;
  padding: 8px 12px;
  border-bottom: 1px solid #e0e0e0;
  font-size: 14px;
}

.log-entry:nth-child(even) {
  background-color: rgba(0, 0, 0, 0.02);
}

.log-time {
  min-width: 90px;
  color: #666;
}

.log-level {
  min-width: 80px;
  font-weight: bold;
}

.log-info .log-level {
  color: #0277bd;
}

.log-warning .log-level {
  color: #ef6c00;
}

.log-error .log-level {
  color: #c62828;
}

.log-message {
  flex: 1;
}

.empty-logs {
  padding: 24px;
  text-align: center;
  color: #666;
  font-style: italic;
}

/* Toast notifications */
.toast-container {
  position: fixed;
  bottom: 24px;
  right: 24px;
  z-index: 1001;
  display: flex;
  flex-direction: column;
  gap: 8px;
  max-width: 400px;
}

.toast {
  background-color: white;
  border-radius: 4px;
  box-shadow: 0 2px 10px rgba(0, 0, 0, 0.15);
  padding: 12px 16px;
  display: flex;
  align-items: center;
  justify-content: space-between;
  animation: slideIn 0.3s ease-out;
}

.toast-content {
  display: flex;
  align-items: center;
  gap: 12px;
}

.toast-icon {
  font-size: 20px;
}

.toast-message {
  font-size: 14px;
}

.toast-close {
  background: none;
  border: none;
  font-size: 18px;
  cursor: pointer;
  color: #666;
  margin-left: 12px;
}

.toast.success {
  border-left: 4px solid #4caf50;
}

.toast.error {
  border-left: 4px solid #f44336;
}

.toast.warning {
  border-left: 4px solid #ff9800;
}

.toast.info {
  border-left: 4px solid #2196f3;
}

@keyframes slideIn {
  from {
    transform: translateX(100%);
    opacity: 0;
  }
  to {
    transform: translateX(0);
    opacity: 1;
  }
}

/* Responsive adjustments */
@media (max-width: 768px) {
  .devices-list {
    grid-template-columns: 1fr;
  }
  
  .sensor-grid {
    grid-template-columns: 1fr 1fr;
  }
  
  .action-buttons {
    flex-direction: column;
  }
  
  .action-button {
    width: 100%;
  }
}



.power-off-info {
  background-color: #fff3e0;
  border-left: 4px solid #ff9800;
  padding: 12px 16px;
  margin-top: 16px;
}

.power-off-info h4 {
  margin-top: 0;
  margin-bottom: 8px;
  color: #ef6c00;
}

.power-off-info ul {
  margin: 0;
  padding-left: 20px;
}

.power-off-info li {
  margin-bottom: 4px;
  color: #bf360c;
}

.confirm-button.power-off {
  background-color: #d32f2f;
}

.confirm-button.power-off:hover:not(:disabled) {
  background-color: #b71c1c;
}






</style>