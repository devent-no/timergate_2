<template>
  <div class="timer-container">
    <!-- Status og hovedklokke -->
    <div class="status-indicator" :class="statusClass">
      {{ statusText }}
    </div>

    <div class="timer-display" :class="statusClass">
      <div class="time">{{ formattedTime }}</div>
      <div class="penalties" v-if="!isEliminated">
        <div class="penalty-badge faults" v-if="faults > 0">
          <div class="fault-icon">✋</div>
          <div class="penalty-count">{{ faults }}</div>
        </div>
        <div class="penalty-badge refusals" v-if="refusals > 0">
          <div class="refusal-icon">✊</div>
          <div class="penalty-count">{{ refusals }}</div>
        </div>
      </div>
      <div class="eliminated-banner" v-if="isEliminated">
        DISQUALIFIED
      </div>
    </div>

    <!-- Kontrollpanel -->
    <div class="control-panel">
      <button @click="resetTimer" class="reset-button">
        <span class="button-icon">↺</span>
        <span class="button-text">RESET</span>
      </button>
      <button v-if="status !== 'ready'" @click="calibrateSensors" class="calibrate-button">
        <span class="button-icon">⚙️</span>
        <span class="button-text">CALIBRATE</span>
      </button>
    </div>

    <!-- Straffepoeng kontroller -->
    <div class="penalty-controls">
      <div class="penalty-group">
        <div class="penalty-header">
          <div class="fault-icon">✋</div>
          <span class="penalty-label">Faults</span>
        </div>
        <div class="penalty-buttons">
          <button @click="decreaseFaults" class="penalty-button minus" :disabled="faults <= 0">−</button>
          <span class="penalty-value">{{ faults }}</span>
          <button @click="increaseFaults" class="penalty-button plus">+</button>
        </div>
      </div>
      
      <div class="penalty-group">
        <div class="penalty-header">
          <div class="refusal-icon">✊</div>
          <span class="penalty-label">Refusals</span>
        </div>
        <div class="penalty-buttons">
          <button @click="decreaseRefusals" class="penalty-button minus" :disabled="refusals <= 0">−</button>
          <span class="penalty-value">{{ refusals }}</span>
          <button @click="increaseRefusals" class="penalty-button plus">+</button>
        </div>
      </div>
      

      <div class="penalty-group">
      <div class="penalty-header">
        <div class="disqualify-icon">❌</div>
        <span class="penalty-label"></span>
      </div>
      <button 
        @click="toggleElimination" 
        class="eliminate-button"
        :class="{ 'eliminated': isEliminated }"
      >
        <span class="button-text">{{ isEliminated ? 'REMOVE DQ' : 'DISQUALIFY' }}</span>
      </button>
    </div>


    </div>

    <!-- Siste tider -->
    <div class="recent-times">
      <h3>Siste resultater</h3>
      <div class="times-list">
        <div v-for="(time, index) in recentTimes" :key="index" class="time-entry">
          <span class="time-number">{{ index + 1 }}</span>
          <span class="time-value">{{ formatTime(time.time) }}</span>
          <div class="time-penalties-container">
            <span class="time-penalty-item" v-if="time.faults">
              <span class="fault-icon small">✋</span> {{ time.faults }}
            </span>
            <span class="time-penalty-item" v-if="time.refusals">
              <span class="refusal-icon small">✊</span> {{ time.refusals }}
            </span>
            <span class="time-eliminated" v-if="time.eliminated">DQ</span>
          </div>
        </div>
        <div v-if="recentTimes.length === 0" class="no-times">
          Ingen resultater registrert ennå
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
    breaks: {
      type: Object,
      default: () => ({})
    },
    passages: {
      type: Array,
      default: () => []
    },
    // Tilkoblingsstatus fra foreldre-komponenten
    connected: {
      type: Boolean,
      default: true
    },
    // Sensorer OK fra foreldre-komponenten
    sensorsOk: {
      type: Boolean,
      default: true
    }
  },
  data() {
    return {
      status: 'ready', // 'ready', 'running', 'finished', 'error'
      currentTime: 0,
      timerRunning: false,
      faults: 0,
      refusals: 0,
      isEliminated: false,
      recentTimes: [], // Array av {time, faults, refusals, eliminated}
      timerStartTime: 0,
      lastPassageTime: 0
    };
  },
  computed: {
    statusText() {
      switch (this.status) {
        case 'ready':
          return 'READY';
        case 'running':
          return 'RUNNING';
        case 'finished':
          return 'FINISHED';
        case 'error':
          return 'ERROR!';
        default:
          return 'READY';
      }
    },
    statusClass() {
      return {
        'status-ready': this.status === 'ready',
        'status-running': this.status === 'running',
        'status-finished': this.status === 'finished',
        'status-error': this.status === 'error'
      };
    },
    formattedTime() {
      if (this.currentTime === 0) {
        return '00:00.00';
      }
      
      const totalMs = this.currentTime;
      const minutes = Math.floor(totalMs / 60000);
      const seconds = Math.floor((totalMs % 60000) / 1000);
      const hundredths = Math.floor((totalMs % 1000) / 10);
      
      return `${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}.${hundredths.toString().padStart(2, '0')}`;
    },
    // Beregn de 5 siste tidene fra passages-prop
    lastTimes() {
      if (!this.passages || this.passages.length === 0) {
        return [];
      }
      
      return [...this.passages]
        .sort((a, b) => b.time - a.time)
        .slice(0, 5);
    }
  },
  watch: {
    // Observatør for passages - oppdaterer timer og siste tider når nye passeringer registreres
    passages: {
    handler(newPassages, oldPassages) {
      if (newPassages.length > 0 && (!oldPassages || newPassages.length > oldPassages.length)) {
        // En ny passering er lagt til
        const latestPassage = newPassages[newPassages.length - 1];
        
        if (this.status === 'running') {
          // Hvis klokken allerede kjører, stopp den (målpassering)
          this.currentTime = latestPassage.time - this.timerStartTime;
          this.stopTimer();
          this.status = 'finished';
          
          // Legg til i nylige tider
          this.addCompletedTime();
        } else {
          // Hvis klokken ikke kjører, start den (startpassering)
          this.resetPenalties();
          this.timerStartTime = latestPassage.time;
          this.currentTime = 0;
          this.startTimer();
          this.status = 'running';
        }
      }
    },
    deep: true
  },
    
    // Overvåk tilkoblingsstatus
    connected(isConnected) {
      if (!isConnected && this.status !== 'error') {
        this.status = 'error';
      } else if (isConnected && this.status === 'error' && this.sensorsOk) {
        this.status = this.timerRunning ? 'running' : 'ready';
      }
    },
    
    // Overvåk sensorstatus
    sensorsOk(areOk) {
      if (!areOk && this.status !== 'error') {
        this.status = 'error';
      } else if (areOk && this.status === 'error' && this.connected) {
        this.status = this.timerRunning ? 'running' : 'ready';
      }
    }
  },
  methods: {
    startTimer() {
      this.timerRunning = true;
      this.timerInterval = setInterval(() => {
        const now = Date.now();
        this.currentTime = now - this.timerStartTime;
      }, 10); // Oppdater hvert 10ms for smooth visning
    },
    
    stopTimer() {
      this.timerRunning = false;
      clearInterval(this.timerInterval);
    },
    
    resetTimer() {
      this.stopTimer();
      this.currentTime = 0;
      this.resetPenalties();
      this.status = 'ready';
      //this.status = this.connected && this.sensorsOk ? 'ready' : 'error';
      // Emit en hendelse for å informere foreldre-komponenten
      //this.$emit('reset');
    },
    
    resetPenalties() {
      this.faults = 0;
      this.refusals = 0;
      this.isEliminated = false;
    },
    
    calibrateSensors() {
      // Emit en hendelse som foreldre-komponenten kan lytte etter
      this.$emit('calibrate');
      this.status = 'ready';
    },
    
    increaseFaults() {
      this.faults++;
    },
    
    decreaseFaults() {
      if (this.faults > 0) {
        this.faults--;
      }
    },
    
    increaseRefusals() {
      this.refusals++;
    },
    
    decreaseRefusals() {
      if (this.refusals > 0) {
        this.refusals--;
      }
    },
    
    toggleElimination() {
      this.isEliminated = !this.isEliminated;
    },
    
    addCompletedTime() {
      // Legg til gjeldende tid og feil i listen over siste tider
      this.recentTimes.unshift({
        time: this.currentTime,
        faults: this.faults,
        refusals: this.refusals,
        eliminated: this.isEliminated,
        timestamp: Date.now()
      });
      
      // Behold bare de 5 siste tidene
      if (this.recentTimes.length > 5) {
        this.recentTimes.pop();
      }
    },
    
    formatTime(ms) {
      const minutes = Math.floor(ms / 60000);
      const seconds = Math.floor((ms % 60000) / 1000);
      const hundredths = Math.floor((ms % 1000) / 10);
      
      return `${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}.${hundredths.toString().padStart(2, '0')}`;
    }
  },
  mounted() {
    // Nullstill timer ved oppstart
    this.resetTimer();
    
    // Sett initial status basert på tilkoblingsstatus og sensorstatus
    if (!this.connected || !this.sensorsOk) {
      this.status = 'error';
    }
  },
  beforeDestroy() {
    // Rydd opp ved komponent-ødeleggelse
    if (this.timerInterval) {
      clearInterval(this.timerInterval);
    }
  }
};
</script>

<style scoped>
.timer-container {
  display: flex;
  flex-direction: column;
  align-items: center;
  width: 100%;
  max-width: 800px;
  margin: 0 auto;
  padding: 1rem;
  font-family: 'Arial', sans-serif;
  color: #333;
}

/* Status-indikator */
.status-indicator {
  font-size: 1.5rem;
  font-weight: bold;
  padding: 0.5rem 2rem;
  border-radius: 6px;
  margin-bottom: 1.5rem;
  text-align: center;
  max-width: 200px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
  text-transform: uppercase;
  letter-spacing: 1px;
}

.status-ready {
  background-color: #4CAF50;
  color: white;
}

.status-running {
  background-color: #FFC107;
  color: #333;
}

.status-finished {
  background-color: #2196F3;
  color: white;
}

.status-error {
  background-color: #F44336;
  color: white;
  animation: pulse 1.5s infinite;
}

@keyframes pulse {
  0% { box-shadow: 0 0 0 0 rgba(244, 67, 54, 0.7); }
  70% { box-shadow: 0 0 0 6px rgba(244, 67, 54, 0); }
  100% { box-shadow: 0 0 0 0 rgba(244, 67, 54, 0); }
}

/* Timer-display */
.timer-display {
  width: 100%;
  text-align: center;
  padding: 2rem;
  margin-bottom: 1.5rem;
  border-radius: 8px;
  border: 1px solid #E0E0E0;
  background-color: #FFFFFF;
  box-shadow: 0 4px 10px rgba(0, 0, 0, 0.1);
}

.timer-display .time {
  font-size: 5rem;
  font-weight: bold;
  font-family: 'Roboto Mono', monospace;
  margin-bottom: 1rem;
  color: #333;
  letter-spacing: 2px;
}

.timer-display .penalties {
  display: flex;
  justify-content: center;
  gap: 1.5rem;
  min-height: 4rem; /* Fast høyde for penalty-området */
}

.timer-display .eliminated-banner {
  min-height: 4rem; /* Samme høyde som penalties */
  display: flex;
  align-items: center;
  justify-content: center;
}

.penalty-badge {
  display: flex;
  align-items: center;
  padding: 0.5rem 1rem;
  border-radius: 20px;
  font-size: 1.2rem;
}

.penalty-badge.faults {
  background-color: #FFF3E0;
  color: #E65100;
  border: 1px solid #FFE0B2;
}

.penalty-badge.refusals {
  background-color: #E3F2FD;
  color: #0D47A1;
  border: 1px solid #BBDEFB;
}

.fault-icon, .refusal-icon {
  font-size: 1.4rem;
  margin-right: 6px;
}

.penalty-count {
  font-size: 1.4rem;
  font-weight: bold;
}

.eliminated-banner {
  background-color: #FFEBEE;
  color: #C62828;
  border: 1px solid #FFCDD2;
  padding: 0.8rem 1.5rem;
  border-radius: 6px;
  font-size: 1.5rem;
  font-weight: bold;
  margin-top: 0.5rem;
}

/* Kontrollpanel */
.control-panel {
  display: flex;
  justify-content: center;
  gap: 1.5rem;
  margin-bottom: 2rem;
  width: 100%;
}

.reset-button, .calibrate-button {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 8px;
  font-size: 1rem;
  font-weight: bold;
  padding: 0.8rem 1.5rem;
  border: none;
  border-radius: 6px;
  cursor: pointer;
  transition: all 0.2s ease;
}

.reset-button {
  background-color: #F44336;
  color: white;
}

.reset-button:hover {
  background-color: #D32F2F;
  transform: translateY(-2px);
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
}

.calibrate-button {
  background-color: #2196F3;
  color: white;
}

.calibrate-button:hover {
  background-color: #1976D2;
  transform: translateY(-2px);
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
}

.button-icon {
  font-size: 1.2rem;
}

/* Straffepoeng kontroller */
.penalty-controls {
  display: flex;
  flex-wrap: wrap;
  justify-content: center;
  gap: 1rem;
  margin-bottom: 2rem;
  width: 100%;
}

.penalty-group {
  display: flex;
  flex-direction: column;
  align-items: center;
  padding: 1rem;
  background-color: #FFFFFF;
  border: 1px solid #E0E0E0;
  border-radius: 8px;
  min-width: 140px;
}

.penalty-header {
  display: flex;
  align-items: center;
  margin-bottom: 0.8rem;
}

.penalty-label {
  font-weight: bold;
  font-size: 1.1rem;
  margin-left: 6px;
  color: #424242;
}

.penalty-buttons {
  display: flex;
  align-items: center;
  gap: 10px;
}

.penalty-button {
  width: 2rem;
  height: 2rem;
  font-size: 1.2rem;
  font-weight: bold;
  border: none;
  border-radius: 50%;
  cursor: pointer;
  transition: all 0.2s ease;
  display: flex;
  align-items: center;
  justify-content: center;
}

.penalty-button.minus {
  background-color: #FFEBEE;
  color: #C62828;
}

.penalty-button.plus {
  background-color: #E8F5E9;
  color: #2E7D32;
}

.penalty-button:hover:not(:disabled) {
  transform: scale(1.1);
}

.penalty-button.minus:hover:not(:disabled) {
  background-color: #FFCDD2;
}

.penalty-button.plus:hover:not(:disabled) {
  background-color: #C8E6C9;
}

.penalty-button:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

.penalty-value {
  width: 1.5rem;
  text-align: center;
  font-size: 1.5rem;
  font-weight: bold;
}

.eliminate-button {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 8px;
  background-color: #FFFFFF; /* Endre dette til #FFFFFF for hvit bakgrunn */
  color: #424242;
  border: 1px solid #E0E0E0;
  font-weight: bold;
  font-size: 0.9rem;
  padding: 0.8rem 1.2rem;
  border-radius: 6px;
  cursor: pointer;
  transition: all 0.2s ease;
}

.eliminate-button:hover {
  background-color: #f1e2e2;
  transform: translateY(-2px);
}

.eliminate-button.eliminated {
  background-color: #FFFFFF; /* Endre dette til #FFFFFF for hvit bakgrunn */
  color: #2E7D32;
  border-color: #C8E6C9;
}

.eliminate-button.eliminated:hover {
  background-color: #FFFFFF;
}

/* Siste tider */
.recent-times {
  width: 100%;
  margin-top: 1rem;
  background-color: #FFFFFF;
  border: 1px solid #E0E0E0;
  border-radius: 8px;
  overflow: hidden;
}

.recent-times h3 {
  font-size: 1.2rem;
  margin: 0;
  padding: 0.8rem;
  text-align: center;
  background-color: #F5F5F5;
  border-bottom: 1px solid #E0E0E0;
  color: #424242;
}

.times-list {
  width: 100%;
  max-height: 250px;
  overflow-y: auto;
}

.time-entry {
  display: flex;
  align-items: center;
  padding: 0.7rem 1rem;
  border-bottom: 1px solid #EEEEEE;
  transition: background-color 0.2s;
}

.time-entry:hover {
  background-color: #FAFAFA;
}

.time-number {
  width: 2rem;
  height: 2rem;
  font-size: 0.9rem;
  font-weight: bold;
  background-color: #F5F5F5;
  color: #424242;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  margin-right: 0.8rem;
}

.time-value {
  flex: 1;
  font-family: 'Roboto Mono', monospace;
  font-size: 1.2rem;
  font-weight: bold;
  color: #333;
}

.time-penalties-container {
  display: flex;
  gap: 0.8rem;
  align-items: center;
}

.time-penalty-item {
  display: flex;
  align-items: center;
  gap: 4px;
  font-size: 0.9rem;
  background-color: #F5F5F5;
  padding: 3px 6px;
  border-radius: 4px;
}

.fault-icon.small, .refusal-icon.small {
  font-size: 0.9rem;
}

.time-eliminated {
  background-color: #FFEBEE;
  color: #C62828;
  padding: 3px 6px;
  border-radius: 4px;
  font-weight: bold;
  font-size: 0.8rem;
}

.no-times {
  text-align: center;
  color: #757575;
  padding: 1.5rem;
  font-style: italic;
}

/* Responsive styling */
@media (max-width: 768px) {
  .timer-display .time {
    font-size: 3.5rem;
  }
  
  .control-panel {
    flex-direction: column;
    align-items: center;
    gap: 0.8rem;
  }
  
  .reset-button, .calibrate-button {
    width: 100%;
    max-width: 250px;
  }
  
  .penalty-controls {
    flex-direction: column;
    align-items: center;
  }
  
  .penalty-group {
    width: 100%;
    max-width: 250px;
  }
}

@media (max-width: 480px) {
  .timer-display .time {
    font-size: 2.8rem;
  }
  
  .status-indicator {
    font-size: 1.2rem;
    padding: 0.4rem 1rem;
  }
  
  .eliminated-banner {
    font-size: 1.2rem;
    padding: 0.5rem 1rem;
  }
}

.disqualify-icon {
  font-size: 1.4rem;
  display: flex;
  justify-content: center;
  margin-bottom: 8px;
  width: 100%;
  text-align: center;
}

.penalty-group {
  display: flex;
  flex-direction: column;
  align-items: center;
  padding: 1rem;
  background-color: #FFFFFF;
  border: 1px solid #E0E0E0;
  border-radius: 8px;
  min-width: 140px;
}

.eliminate-button {
  background-color: #FFFFFF; /* Hvit bakgrunn */
  color: #424242;
  border: 1px solid #E0E0E0;
  font-weight: bold;
  font-size: 0.9rem;
  padding: 0.8rem 1.2rem;
  border-radius: 6px;
  cursor: pointer;
  transition: all 0.2s ease;
  /* Fjern flex-relaterte egenskaper som tidligere var her */
}
</style>