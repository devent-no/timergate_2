<template>
  <div class="pole-status-container">
    <div class="pole-status-indicators">
      <!-- Kun vis faktiske tilkoblede målestolper -->
      <div 
        v-for="pole in activePoles" 
        :key="pole.mac"
        class="pole-indicator"
        :class="getPoleStatusClass(pole)"
        :title="getPoleTooltip(pole)"
      >
        <div class="pole-name">{{ pole.displayName }}</div>
        <div class="pole-sensors">
          {{ pole.activeSensors }}/{{ pole.totalSensors }}
          <!-- Vis countdown hvis relevant -->
          <span v-if="warningTimers[pole.mac] && warningTimers[pole.mac].countdown > 0" class="countdown">
            ({{ warningTimers[pole.mac].countdown }}s)
          </span>
        </div>
        <div class="pole-signal">
          <span v-if="pole.signalStatus === 'disconnected'" class="disconnected-indicator">⚠️</span>
          <span v-else>{{ pole.rssi }}dBm</span>
        </div>
      </div>
      
      <!-- Vis melding når ingen målestolper -->
      <div v-if="activePoles.length === 0" class="no-poles">
        <span class="no-poles-icon">⚠️</span>
        <span class="no-poles-text">Ingen målestolper</span>
      </div>
    </div>
  </div>
</template>

<script>
export default {
  name: 'PoleStatusIndicator',
  props: {
    serverAddress: {
      type: String,
      default: 'timergate.local'
    },
    // VIKTIG: Bruker poles fra App.vue (via WebSocket)
    poles: {
      type: Array,
      default: () => []
    }
  },
  data() {
    return {
      signalData: {},
      updateInterval: null,
      realTimeSensorData: {}, // Sporer sanntids sensorendringer fra K=1
      warningTimers: {}, // NYTT: Sporer countdown for hver målestolpe
      countdownIntervals: {}, // Holder styr på aktive countdown-intervaller
      criticalAfterCountdown: {} // NYTT: Sporer målestolper som er kritiske etter countdown
    }
  },
  computed: {
    activePoles() {
      // Kombiner pole-data fra WebSocket med signal-data og sanntids sensor-data
      return this.poles.map((pole, index) => {
        const signal = this.signalData[pole.mac] || {};
        
        // FORBEDRET: Bruk sanntids sensor-data hvis tilgjengelig, ellers WebSocket K=0 data
        let broken = pole.broken || [];
        if (this.realTimeSensorData[pole.mac]) {
          // Bruk oppdatert sensor-data fra K=1 meldinger
          broken = [...this.realTimeSensorData[pole.mac]];
        }
        
        // Beregn aktive sensorer fra FAKTISK broken-array
        // VIKTIG: broken[i] = true betyr sensor er OK/AKTIV (ikke brutt)
        // broken[i] = false betyr sensor er BLOKKERT/BRUTT
        const totalSensors = broken.length || 7;
        const activeSensors = broken.filter(isBroken => isBroken).length;
        
        // Bestem visningsnavn
        const displayName = pole.name || `${String.fromCharCode(65 + index)}`;
        
        // Bestem signal-status
        let signalStatus = 'disconnected';
        let signalQuality = 0;
        let rssi = -999;
        
        if (signal.rssi) {
          rssi = signal.rssi;
          signalQuality = signal.quality || 0;
          
          // Sjekk om signal er for gammelt (mer enn 10 sekunder)
          const now = Date.now();
          const signalAge = signal.lastUpdate ? (now - signal.lastUpdate * 1000) : Infinity;
          
          if (signalAge < 10000) { // Mindre enn 10 sekunder
            signalStatus = 'connected';
          }
        }
        
        return {
          mac: pole.mac,
          displayName,
          activeSensors,
          totalSensors,
          signalStatus,
          signalQuality,
          rssi,
          rawData: pole // For debugging
        };
      });
    }
  },
  mounted() {
    this.startSignalUpdates();
    this.listenForSensorChanges();
  },
  beforeUnmount() {
    this.stopSignalUpdates();
    this.stopAllCountdowns();
  },
  methods: {
    startSignalUpdates() {
      // Hent initial signal data
      this.updateSignalData();
      
      // Oppdater signal data hvert 5. sekund
      this.updateInterval = setInterval(() => {
        this.updateSignalData();
      }, 5000);
    },
    
    stopSignalUpdates() {
      if (this.updateInterval) {
        clearInterval(this.updateInterval);
        this.updateInterval = null;
      }
    },
    
    stopAllCountdowns() {
      // Stopp alle aktive countdown-intervaller
      Object.values(this.countdownIntervals).forEach(interval => {
        clearInterval(interval);
      });
      this.countdownIntervals = {};
    },
    
    // NYTT: Lytter på WebSocket meldinger fra parent (App.vue)
    listenForSensorChanges() {
      // Dette kalles automatisk via watch på props.poles
      // Men vi kan også lytte direkte på WebSocket hvis nødvendig
    },
    
    // FORBEDRET: Håndter K=1 sensor-endringer med countdown-logikk
    updateSensorFromBreak(mac, sensorId, breakState) {
      if (!this.realTimeSensorData[mac]) {
        // Initialiser med current state hvis ikke eksisterende
        const pole = this.poles.find(p => p.mac === mac);
        this.realTimeSensorData[mac] = pole?.broken ? [...pole.broken] : new Array(7).fill(true);
      }
      
      // FORBEDRET: Validér sensor ID
      if (sensorId < 0 || sensorId >= 7) {
        console.warn(`Ugyldig sensor ID: ${sensorId} for MAC ${mac}`);
        return;
      }
      
      // Oppdater spesifikk sensor
      // breakState = 1 betyr brudd (sensor blokkert) = false i broken array
      // breakState = 0 betyr ok (sensor aktiv) = true i broken array
      const wasActive = this.realTimeSensorData[mac][sensorId];
      this.realTimeSensorData[mac][sensorId] = (breakState === 0);
      
      console.log(`Sensor ${sensorId} på ${mac}: ${breakState === 1 ? 'BLOKKERT' : 'AKTIV'}`, {
        sensorId,
        breakState,
        wasActive,
        nowActive: this.realTimeSensorData[mac][sensorId],
        allSensors: this.realTimeSensorData[mac]
      });
      
      // NYTT: Sjekk countdown-logikk basert på oppdaterte sensorer
      this.checkCountdownLogic(mac);
      
      // Trigger Vue reactivity update
      this.$forceUpdate();
    },
    
    // NYTT: Sjekk om countdown skal startes/stoppes
    checkCountdownLogic(mac) {
      const pole = this.activePoles.find(p => p.mac === mac);
      if (!pole) return;
      
      const blockedSensors = pole.totalSensors - pole.activeSensors;
      
      if (blockedSensors >= 2) {
        // Start countdown hvis ikke allerede aktiv
        if (!this.warningTimers[mac]) {
          console.log(`🟠 Starter 5-sekunders countdown for ${mac} (${blockedSensors} sensorer blokkert)`);
          this.startCountdown(mac);
        }
      } else {
        // Stopp countdown og reset kritisk tilstand hvis aktiv
        if (this.warningTimers[mac]) {
          console.log(`🟢 Stopper countdown for ${mac} (kun ${blockedSensors} sensorer blokkert)`);
          this.stopCountdown(mac);
        }
        // Reset kritisk tilstand når færre enn 2 sensorer er blokkert
        if (this.criticalAfterCountdown[mac]) {
          delete this.criticalAfterCountdown[mac];
          console.log(`🟢 Resetter kritisk tilstand for ${mac}`);
        }
      }
    },
    
    // NYTT: Start 5-sekunders countdown
    startCountdown(mac) {
      this.warningTimers[mac] = {
        startTime: Date.now(),
        countdown: 5
      };
      
      this.countdownIntervals[mac] = setInterval(() => {
        if (this.warningTimers[mac]) {
          const elapsed = (Date.now() - this.warningTimers[mac].startTime) / 1000;
          const remaining = Math.max(0, 5 - elapsed);
          this.warningTimers[mac].countdown = Math.ceil(remaining);
          
          if (remaining <= 0) {
            console.log(`🔴 Countdown ferdig for ${mac} - aktiverer kritisk tilstand (rødt blink)`);
            this.criticalAfterCountdown[mac] = true; // Aktiver kritisk tilstand
            this.stopCountdown(mac);
          }
        } else {
          this.stopCountdown(mac);
        }
      }, 100);
    },
    
    // NYTT: Stopp countdown for spesifikk målestolpe
    stopCountdown(mac) {
      if (this.countdownIntervals[mac]) {
        clearInterval(this.countdownIntervals[mac]);
        delete this.countdownIntervals[mac];
      }
      if (this.warningTimers[mac]) {
        delete this.warningTimers[mac];
      }
    },
    
    async updateSignalData() {
      try {
        const response = await fetch(`http://${this.serverAddress}/api/v1/signal/quality`);
        const data = await response.json();
        
        if (data.status === 'success') {
          // Konverter array til objekt med MAC som nøkkel
          const newSignalData = {};
          data.signal_data.forEach(signal => {
            newSignalData[signal.mac] = {
              rssi: signal.rssi,
              quality: signal.quality,
              lastUpdate: signal.last_update
            };
          });
          
          this.signalData = newSignalData;
        }
      } catch (error) {
        console.error('Feil ved henting av signal-data:', error);
        // Ved feil, behold forrige signal-data
      }
    },
    
    // FORBEDRET: Hierarkisk statuslogikk som reflekterer målestolpens tilstand
    getPoleStatusClass(pole) {
      // 1. KRITISK: Frakoblet
      if (pole.signalStatus === 'disconnected') {
        return 'status-disconnected'; // GRÅ
      }
      
      // 2. KRITISK: Rødt blink-tilstand (samme som målestolpe)
      // Sjekk både direkte 0 sensorer OG etter ferdig countdown
      if (pole.activeSensors === 0 || this.criticalAfterCountdown[pole.mac]) {
        return 'status-critical-blocked'; // RØD BLINKENDE
      }
      
      // 3. ADVARSEL: Delvis blokkert
      if (pole.activeSensors < pole.totalSensors) {
        const blockedSensors = pole.totalSensors - pole.activeSensors;
        
        if (blockedSensors >= 2) {
          return 'status-warning-critical'; // ORANGE (kan bli kritisk om 5 sek)
        } else {
          return 'status-warning-minor'; // GUL (1 sensor blokkert)
        }
      }
      
      // 4. OK: Alle sensorer aktive - basert på signalkvalitet
      if (pole.signalQuality >= 70) return 'status-good';           // GRØNN
      if (pole.signalQuality >= 50) return 'status-fair';          // LYS GUL  
      if (pole.signalQuality >= 30) return 'status-poor';          // ORANGE
      return 'status-critical-signal';                             // RØD (dårlig signal)
    },
    
    // FORBEDRET: Tooltip med mer detaljert informasjon
    getPoleTooltip(pole) {
      if (pole.signalStatus === 'disconnected') {
        return `${pole.displayName}: Frakoblet`;
      }
      
      const blockedSensors = pole.totalSensors - pole.activeSensors;
      let status = '';
      
      if (pole.activeSensors === 0 || this.criticalAfterCountdown[pole.mac]) {
        status = 'KRITISK - Rødt blink aktivt!';
      } else if (blockedSensors >= 2) {
        const countdown = this.warningTimers[pole.mac]?.countdown;
        status = `ADVARSEL - ${blockedSensors} sensorer blokkert${countdown ? ` (${countdown}s til kritisk)` : ' (kan bli kritisk)'}`;
      } else if (blockedSensors === 1) {
        status = 'MINDRE BEKYMRING - 1 sensor blokkert';
      } else {
        status = 'OK - Alle sensorer aktive';
      }
      
      return `${pole.displayName}: ${status}\n` +
             `Aktive sensorer: ${pole.activeSensors}/${pole.totalSensors}\n` +
             `Signal: ${pole.rssi}dBm (kvalitet: ${pole.signalQuality}%)`;
    }
  }
}
</script>

<style scoped>
.pole-status-container {
  background: white;
  border-radius: 8px;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.1);
  padding: 12px;
  min-width: 280px;
  max-width: 320px;
  margin: 8px;
}

.pole-status-indicators {
  display: flex;
  flex-direction: column;
  gap: 8px;
}

.pole-indicator {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 12px;
  border-radius: 6px;
  border: 1px solid #e0e0e0;
  transition: all 0.3s ease;
}

.pole-indicator:hover {
  transform: translateY(-1px);
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.15);
}

/* HIERARKISK STATUS-SYSTEM */

/* 1. KRITISK TILSTANDER */
.status-disconnected {
  background-color: #f5f5f5;
  border-left: 4px solid #9e9e9e;
  color: #616161;
  opacity: 0.7;
}

.status-critical-blocked {
  background-color: #d32f2f;
  border-left: 4px solid #b71c1c;
  color: white;
  font-weight: bold;
  animation: pulse-critical 1s infinite;
}

@keyframes pulse-critical {
  0% { opacity: 1; }
  50% { opacity: 0.7; }
  100% { opacity: 1; }
}

.status-critical-signal {
  background-color: #ffebee;
  border-left: 4px solid #d32f2f;
  color: #c62828;
  font-weight: bold;
}

/* 2. ADVARSEL TILSTANDER */
.status-warning-critical {
  background-color: #fff3e0;
  border-left: 4px solid #ff9800;
  color: #f57c00;
  font-weight: 600;
}

.status-warning-minor {
  background-color: #fffde7;
  border-left: 4px solid #ffc107;
  color: #f9a825;
}

.status-poor {
  background-color: #fff3e0;
  border-left: 4px solid #ff9800;
  color: #f57c00;
}

/* 3. OK TILSTANDER */
.status-good {
  background-color: #e8f5e9;
  border-left: 4px solid #4caf50;
  color: #2e7d32;
}

.status-fair {
  background-color: #fffde7;
  border-left: 4px solid #cddc39;
  color: #827717;
}

/* LAYOUT ELEMENTER */
.pole-name {
  font-weight: 600;
  width: 60px;
  text-align: left;
}

.pole-sensors {
  font-weight: 700;
  width: 60px;
  text-align: center;
  position: relative;
}

.countdown {
  display: block;
  font-size: 10px;
  font-weight: 500;
  color: #ff5722;
  margin-top: 2px;
}

.pole-signal {
  font-size: 12px;
  opacity: 0.8;
  width: 60px;
  text-align: right;
  display: flex;
  align-items: center;
  justify-content: flex-end;
}

.disconnected-indicator {
  font-size: 14px;
  color: #f44336;
}

.no-poles {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 16px;
  color: #666;
  font-style: italic;
  justify-content: center;
}

.no-poles-icon {
  font-size: 16px;
}

/* Responsiv design for header */
@media (max-width: 768px) {
  .pole-status-container {
    min-width: 240px;
    max-width: 280px;
  }
  
  .pole-name {
    width: 50px;
  }
  
  .pole-signal {
    width: 50px;
  }
  
  .pole-sensors {
    width: 50px;
  }
}
</style>