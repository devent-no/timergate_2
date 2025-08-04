<template>
  <div class="timer-container">
    <!-- Status-indikator - kun ved feil -->
    <div v-if="status === 'error'" class="status-indicator" :class="statusClass">
      {{ statusText }}
    </div>

    <!-- Hovedinnhold: Timer til venstre, Logg til høyre -->
    <div class="main-content">
      <!-- Timer-seksjon -->
      <div class="timer-section">
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

        <!-- Store touch-vennlige penalty-kontroller -->
        <div class="penalty-controls-touch">
          <!-- Faults -->
          <div class="penalty-group-touch">
            <div class="penalty-header-touch">
              <div class="fault-icon-large">✋</div>
              <span class="penalty-label-large">Faults</span>
            </div>
            <div class="penalty-buttons-touch">
              <button @click="decreaseFaults" class="penalty-button-touch minus" :disabled="faults <= 0">
                <span class="button-symbol">−</span>
              </button>
              <div class="penalty-value-large">{{ faults }}</div>
              <button @click="increaseFaults" class="penalty-button-touch plus">
                <span class="button-symbol">+</span>
              </button>
            </div>
          </div>
          
          <!-- Refusals -->
          <div class="penalty-group-touch">
            <div class="penalty-header-touch">
              <div class="refusal-icon-large">✊</div>
              <span class="penalty-label-large">Refusals</span>
            </div>
            <div class="penalty-buttons-touch">
              <button @click="decreaseRefusals" class="penalty-button-touch minus" :disabled="refusals <= 0">
                <span class="button-symbol">−</span>
              </button>
              <div class="penalty-value-large">{{ refusals }}</div>
              <button @click="increaseRefusals" class="penalty-button-touch plus">
                <span class="button-symbol">+</span>
              </button>
            </div>
          </div>

          <!-- Disqualification -->
          <div class="penalty-group-touch">
            <div class="penalty-header-touch">
              <div class="disqualify-icon-large">🙅</div>
            </div>
            <button 
              @click="toggleElimination" 
              class="eliminate-button-touch"
              :class="{ 'eliminated': isEliminated }"
            >
              {{ isEliminated ? 'REMOVE DQ' : 'DISQUALIFY' }}
            </button>
          </div>
        </div>

        <!-- Kompakt reset-knapp -->
        <div class="control-panel-compact">
          <button @click="resetTimer" class="reset-button-compact">
            <span class="button-icon-small">↺</span>
            <span class="button-text-small">RESET</span>
          </button>
        </div>
      </div>

      <!-- Logg-seksjon -->
      <div class="log-section">
        <div class="log-header">
          <h3>Siste resultater</h3>
          <div class="log-controls">
            <button @click="clearAllResults" class="log-button clear-all" :disabled="recentTimes.length === 0">
              <span class="icon">🗑️</span>
              Tøm alle
            </button>
          </div>
        </div>
        <div class="times-list-compact">
          <div v-for="(time, index) in recentTimes" :key="time.id" class="time-entry-compact">
            <div class="time-details-compact">
              <div class="time-main-info">
                <span class="time-of-day">{{ time.timeOfDay }}</span>
                <span class="time-value-large">{{ formatTime(time.time) }}</span>
              </div>
              <div class="time-penalties-row">
                <span class="simple-delete" @click="removeResult(index)">🗑️</span>
                <div class="time-penalties">
                  <span class="penalty-item" v-if="time.faults">
                    <span class="fault-icon small">✋</span> {{ time.faults }}
                  </span>
                  <span class="penalty-item" v-if="time.refusals">
                    <span class="refusal-icon small">✊</span> {{ time.refusals }}
                  </span>
                  <span class="eliminated-indicator" v-if="time.eliminated">DQ</span>
                  <span v-if="!time.faults && !time.refusals && !time.eliminated" class="clean-run">Clean</span>
                </div>
              </div>
            </div>
          </div>

          <div v-if="recentTimes.length === 0" class="no-times">
            Ingen resultater registrert ennå
          </div>
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
    serverAddress: {
      type: String,
      default: "timergate.local"
    },
    connected: {
      type: Boolean,
      default: true
    },
    sensorsOk: {
      type: Boolean,
      default: true
    }
  },
  data() {
    return {
      status: 'ready',
      currentTime: 0,
      timerRunning: false,
      faults: 0,
      refusals: 0,
      isEliminated: false,
      recentTimes: [],
      timerStartTime: 0,
      lastPassageTime: 0,
      lastKnownPassagesLength: 0,
      nextResultId: 1,
      // NYE VARIABLER for å håndtere navigasjon
      componentActive: false,  // Sporer om komponenten er aktiv
      passagesLengthAtMount: 0, // Lengde på passages når komponenten mountes
      passagesSnapshot: []  // Snapshot av passages ved mounting
    };
  },
  computed: {
    statusText() {
      switch (this.status) {
        case 'ready': return 'READY';
        case 'running': return 'RUNNING';
        case 'finished': return 'FINISHED';
        case 'error': return 'ERROR!';
        default: return 'READY';
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
    }
  },

  watch: {
    // FORBEDRET: Observatør for passages med bedre navigasjonshåndtering
    passages: {
      handler(newPassages, oldPassages) {
        console.log("🕒 TIMER WATCH FULL DEBUG:", { 
          nyLengde: newPassages?.length,
          gamleLengde: oldPassages?.length,
          lastKnownLength: this.lastKnownPassagesLength,
          actualNew: newPassages.length - this.lastKnownPassagesLength,
          componentActive: this.componentActive,
          status: this.status,
          currentTime: this.currentTime,
          timerRunning: this.timerRunning,
          timerStartTime: this.timerStartTime,
          // Legg til sammenligning av array-innhold
          newPassagesLastItem: newPassages?.length > 0 ? newPassages[newPassages.length - 1] : null,
          oldPassagesLastItem: oldPassages?.length > 0 ? oldPassages[oldPassages.length - 1] : null,
          // Informasjon om array-innhold endring
          contentChanged: JSON.stringify(newPassages) !== JSON.stringify(oldPassages)
        });

        
        // FIKSET: Ignorer endringer hvis komponenten ikke er aktiv
        if (!this.componentActive) {
          console.log("🚫 TIMER: Komponenten er ikke aktiv, ignorerer endringer i passages");
          return;
        }
        
        // FIKSET: Sjekk om dette er en EKTE ny passering eller bare navigasjon
        const actualNewPassages = newPassages.length - this.lastKnownPassagesLength;
        
        if (actualNewPassages > 0) {
          console.log(`🔔 ${actualNewPassages} EKTE NYE PASSERINGER DETEKTERT`);
          
          // Få den nyeste passeringen
          const latestPassage = newPassages[newPassages.length - 1];
          
          if (this.status === 'running') {
            console.log("⏹️ STOPPER TIMER: Timer kjører, stopping ved målpassering");
            this.currentTime = Date.now() - this.timerStartTime;
            this.stopTimer();
            this.status = 'finished';
            this.addCompletedTime();
            this.saveTimerState(); // Lagre tilstand
          } else if (this.status === 'ready' || this.status === 'finished') {
            console.log("▶️ STARTER TIMER: Timer starter ved startpassering");
            this.resetPenalties();
            this.timerStartTime = Date.now();
            this.currentTime = 0;
            this.startTimer();
            this.status = 'running';
            this.saveTimerState(); // Lagre tilstand
          }
          
          // Oppdater sist kjente lengde
          this.lastKnownPassagesLength = newPassages.length;
        // } else if (actualNewPassages < 0) {
        //   console.log("⚠️ TIMER: Passages-array ble kortere, trolig navigasjonsrelatert");
        //   // Juster lastKnownPassagesLength til gjeldende lengde
        //   this.lastKnownPassagesLength = newPassages.length;
        // } else {
        //   console.log("ℹ️ TIMER: Ingen nye passeringer, sannsynligvis navigasjon eller gjenmontering");
        // }


      } else if (actualNewPassages < 0) {
        console.log("⚠️ TIMER: Passages-array ble kortere - FULL DEBUG:", {
          forrigeLength: this.lastKnownPassagesLength,
          nyLength: newPassages.length,
          forskjell: actualNewPassages,
          timerStatus: this.status,
          timerRunning: this.timerRunning,
          currentTime: this.currentTime,
          componentActive: this.componentActive,
          // Sammenlign array-innhold
          newPassages: newPassages.map(p => ({ mac: p.mac, time: p.time, sensors: p.sensors })),
          oldPassages: oldPassages?.map(p => ({ mac: p.mac, time: p.time, sensors: p.sensors })) || []
        });
        
        // Juster lastKnownPassagesLength til gjeldende lengde
        this.lastKnownPassagesLength = newPassages.length;
        
        // NYTT: Sjekk om timer er i en inkonsistent tilstand og reset hvis nødvendig
        if (this.status === 'running' && this.currentTime === 0) {
          console.log("🔄 TIMER: Oppdaget inkonsistent tilstand - resetter timer");
          this.resetTimer();
        }
      }


      },
      deep: true,
      immediate: false  // FIKSET: Ikke kjør umiddelbart ved mounting
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
    },
    
    // Nye watches for straffer
    faults(newValue) {
      if (this.status === 'finished' && this.recentTimes.length > 0) {
        this.recentTimes[0].faults = newValue;
      }
    },
    
    refusals(newValue) {
      if (this.status === 'finished' && this.recentTimes.length > 0) {
        this.recentTimes[0].refusals = newValue;
      }
    },
    
    isEliminated(newValue) {
      if (this.status === 'finished' && this.recentTimes.length > 0) {
        this.recentTimes[0].eliminated = newValue;
      }
    }
  },

  methods: {
    startTimer() {
      this.timerRunning = true;
      this.timerInterval = setInterval(() => {
        const now = Date.now();
        this.currentTime = now - this.timerStartTime;
      }, 10);
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
      
      // NYTT: Lagre tilbakestilt tilstand
      this.saveTimerState();
    },
    
    resetPenalties() {
      this.faults = 0;
      this.refusals = 0;
      this.isEliminated = false;
    },
    
    // NYTT: Separat funksjon for å lagre bare timer-tilstand
    saveTimerState() {
      try {
        const timerState = {
          status: this.status,
          timerRunning: this.timerRunning,
          timerStartTime: this.timerStartTime,
          currentTime: this.currentTime,
          faults: this.faults,
          refusals: this.refusals,
          isEliminated: this.isEliminated,
          lastKnownPassagesLength: this.lastKnownPassagesLength,
          timestamp: Date.now()
        };
        localStorage.setItem('timergate_timer_state', JSON.stringify(timerState));
      } catch (error) {
        console.warn('Kunne ikke lagre timer-tilstand:', error);
      }
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
      if (this.passages && this.passages.length > 0) {
        const latestPassage = this.passages[this.passages.length - 1];
        const passageDate = new Date(latestPassage.time);
        
        const hours = passageDate.getHours().toString().padStart(2, '0');
        const minutes = passageDate.getMinutes().toString().padStart(2, '0');
        const seconds = passageDate.getSeconds().toString().padStart(2, '0');
        const timeOfDay = `${hours}:${minutes}:${seconds}`;
        
        const newResult = {
          id: this.nextResultId++,
          time: this.currentTime,
          faults: this.faults,
          refusals: this.refusals,
          eliminated: this.isEliminated,
          timeOfDay: timeOfDay,
          timestamp: Date.now()
        };
        
        this.recentTimes.unshift(newResult);
        
        if (this.recentTimes.length > 15) {
          this.recentTimes.pop();
        }
        
        this.saveResultsToStorage();
      }
    },
    
    removeResult(index) {
      this.recentTimes.splice(index, 1);
      this.saveResultsToStorage();
    },
    
    clearAllResults() {
      if (confirm('Er du sikker på at du vil fjerne alle resultater fra loggen?')) {
        this.recentTimes = [];
        this.saveResultsToStorage();
      }
    },
    
    saveResultsToStorage() {
      try {
        localStorage.setItem('timergate_results', JSON.stringify(this.recentTimes));
        localStorage.setItem('timergate_next_id', this.nextResultId.toString());
        
        // NYTT: Lagre timer-tilstand
        const timerState = {
          status: this.status,
          timerRunning: this.timerRunning,
          timerStartTime: this.timerStartTime,
          currentTime: this.currentTime,
          faults: this.faults,
          refusals: this.refusals,
          isEliminated: this.isEliminated,
          lastKnownPassagesLength: this.lastKnownPassagesLength,
          timestamp: Date.now() // For å beregne hvor lenge vi var borte
        };
        localStorage.setItem('timergate_timer_state', JSON.stringify(timerState));
      } catch (error) {
        console.warn('Kunne ikke lagre resultater til localStorage:', error);
      }
    },
    
    loadResultsFromStorage() {
      try {
        const savedResults = localStorage.getItem('timergate_results');
        const savedNextId = localStorage.getItem('timergate_next_id');
        
        if (savedResults) {
          this.recentTimes = JSON.parse(savedResults);
        }
        
        if (savedNextId) {
          this.nextResultId = parseInt(savedNextId, 10);
        }
        
        // NYTT: Last inn timer-tilstand
        const savedTimerState = localStorage.getItem('timergate_timer_state');
        if (savedTimerState) {
          const timerState = JSON.parse(savedTimerState);
          const timeSinceLastSave = Date.now() - timerState.timestamp;
          
          console.log("🔄 GJENOPPRETTER TIMER-TILSTAND:", {
            savedStatus: timerState.status,
            timeSinceLastSave: timeSinceLastSave,
            wasRunning: timerState.timerRunning
          });
          
          // Gjenopprett tilstand
          this.status = timerState.status;
          this.faults = timerState.faults;
          this.refusals = timerState.refusals;
          this.isEliminated = timerState.isEliminated;
          //this.lastKnownPassagesLength = timerState.lastKnownPassagesLength;
          
          // FIKSET: Ikke gjenopprett lastKnownPassagesLength - behold faktisk array-lengde
          // this.lastKnownPassagesLength = timerState.lastKnownPassagesLength;
          console.log("📊 BEHOLDER FAKTISK PASSAGES-LENGTH:", {
            savedLength: timerState.lastKnownPassagesLength,
            actualLength: this.lastKnownPassagesLength,
            keepingActual: true
          });

          
          // Hvis timeren var i gang, beregn ny currentTime basert på hvor lenge vi var borte
          if (timerState.timerRunning && timerState.status === 'running') {
            this.timerStartTime = timerState.timerStartTime;
            this.currentTime = timerState.currentTime + timeSinceLastSave;
            this.timerRunning = false; // Vil settes til true i startTimer()
            
            // Start timeren igjen
            this.$nextTick(() => {
              this.startTimer();
            });
            
            console.log("⏰ TIMER GJENOPPRETTET:", {
              originalCurrentTime: timerState.currentTime,
              addedTime: timeSinceLastSave,
              newCurrentTime: this.currentTime
            });
          } else {
            // Timer var ikke i gang
            this.timerStartTime = timerState.timerStartTime;
            this.currentTime = timerState.currentTime;
            this.timerRunning = false;
          }
        }
      } catch (error) {
        console.warn('Kunne ikke laste timer-tilstand fra localStorage:', error);
        this.recentTimes = [];
        this.nextResultId = 1;
        // Ikke endre timer-tilstand ved feil
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
    console.log("🔧 TIMERVIEW MONTERT MED PASSAGES:", {
      passagesLength: this.passages?.length,
      passages: this.passages
    });
    
    // NYTT: Marker komponenten som aktiv
    this.componentActive = true;
    
    // NYTT: Ta snapshot av passages ved mounting
    this.passagesLengthAtMount = this.passages ? this.passages.length : 0;
    this.lastKnownPassagesLength = this.passagesLengthAtMount;
    this.passagesSnapshot = this.passages ? [...this.passages] : [];
    
    console.log("📸 TIMERVIEW: Tatt snapshot ved mounting:", {
      lengde: this.passagesLengthAtMount,
      lastKnownLength: this.lastKnownPassagesLength
    });
    
    // Last inn lagrede resultater
    this.loadResultsFromStorage();
    
    // FIKSET: Ikke start intervall for passages-sjekk (unødvendig)
    this.checkInterval = setInterval(() => {
      // Tom funksjon - kan fjernes i fremtidige versjoner
    }, 5000);
  },

  // NYTT: Lifecycle hook for når komponenten blir deaktivert
  beforeDestroy() {
    console.log("🏁 TIMERVIEW: Komponenten deaktiveres");
    this.componentActive = false;
    
    // VIKTIG: Lagre timer-tilstand før komponenten ødelegges
    this.saveTimerState();
    
    // Rydd opp intervaller
    if (this.timerInterval) {
      clearInterval(this.timerInterval);
    }
    if (this.checkInterval) {
      clearInterval(this.checkInterval);
    }
  },

  // NYTT: For Vue 3 kompatibilitet
  beforeUnmount() {
    console.log("🏁 TIMERVIEW: Komponenten unmountes");
    this.componentActive = false;
    
    // VIKTIG: Lagre timer-tilstand før komponenten ødelegges
    this.saveTimerState();
    
    // Rydd opp intervaller
    if (this.timerInterval) {
      clearInterval(this.timerInterval);
    }
    if (this.checkInterval) {
      clearInterval(this.checkInterval);
    }
  },

  // NYTT: Når komponenten aktiveres igjen (ved navigasjon tilbake)
  activated() {
    console.log("✅ TIMERVIEW: Komponenten aktivert igjen");
    this.componentActive = true;
    
    // FIKSET: Gjenopprett timer hvis den var i gang
    if (this.status === 'running' && !this.timerRunning) {
      console.log("🔄 TIMERVIEW: Gjenoppretter pågående timer");
      this.startTimer();
    }
    
    // Sjekk om det har kommet nye passages mens vi var borte
    const currentLength = this.passages ? this.passages.length : 0;
    console.log("🔄 TIMERVIEW AKTIVERT: Sjekker passages-endringer", {
      tidligereLengde: this.lastKnownPassagesLength,
      gjeldendeLengde: currentLength,
      forskjell: currentLength - this.lastKnownPassagesLength
    });
    
    // Oppdater lastKnownPassagesLength til gjeldende tilstand
    this.lastKnownPassagesLength = currentLength;
  },

  // FIKSET: Når komponenten deaktiveres (ved navigasjon bort)
  deactivated() {
    console.log("⏸️ TIMERVIEW: Komponenten deaktivert - bevarer timer-tilstand");
    this.componentActive = false;
    
    // VIKTIG: Ikke stopp timeren, bare pause interval
    if (this.timerInterval) {
      clearInterval(this.timerInterval);
      this.timerInterval = null;
    }
    // Bevar this.timerRunning = true og this.status = 'running'
  }
};
</script>

<style scoped>
.timer-container {
  display: flex;
  flex-direction: column;
  width: 100%;
  height: 100vh;
  padding: 1rem;
  font-family: 'Arial', sans-serif;
  color: #333;
  box-sizing: border-box;
}

/* Status-indikator - kun ved feil */
.status-indicator {
  font-size: 1.8rem;
  font-weight: bold;
  padding: 0.8rem 2rem;
  border-radius: 8px;
  margin-bottom: 1.5rem;
  text-align: center;
  max-width: 300px;
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.15);
  text-transform: uppercase;
  letter-spacing: 2px;
  align-self: center;
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
  70% { box-shadow: 0 0 0 10px rgba(244, 67, 54, 0); }
  100% { box-shadow: 0 0 0 0 rgba(244, 67, 54, 0); }
}

/* Hovedinnhold: Timer til venstre, Logg til høyre */
.main-content {
  display: flex;
  flex: 1;
  gap: 2rem;
  height: 100%;
  min-height: 0;
}

.timer-section {
  flex: 2.5;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.log-section {
  flex: 1;
  max-width: 350px;
  display: flex;
  flex-direction: column;
  background-color: #FFFFFF;
  border: 1px solid #E0E0E0;
  border-radius: 12px;
  padding: 1.5rem;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.1);
  min-height: 0;
}

/* Timer display */
.timer-display {
  background-color: #FFFFFF;
  border: 2px solid #E0E0E0;
  border-radius: 16px;
  padding: 2rem;
  text-align: center;
  box-shadow: 0 6px 20px rgba(0, 0, 0, 0.1);
}

.timer-display .time {
  font-size: 6rem;
  font-weight: bold;
  font-family: 'Roboto Mono', monospace;
  margin-bottom: 1rem;
  color: #333;
  letter-spacing: 3px;
  line-height: 1;
}

.timer-display .penalties {
  display: flex;
  justify-content: center;
  gap: 2rem;
  min-height: 4rem;
}

.timer-display .eliminated-banner {
  min-height: 4rem;
  display: flex;
  align-items: center;
  justify-content: center;
  background-color: #FFEBEE;
  color: #C62828;
  border: 2px solid #FFCDD2;
  padding: 1rem 2rem;
  border-radius: 8px;
  font-size: 1.8rem;
  font-weight: bold;
  margin-top: 1rem;
}

.penalty-badge {
  display: flex;
  align-items: center;
  padding: 0.8rem 1.5rem;
  border-radius: 25px;
  font-size: 1.5rem;
}

.penalty-badge.faults {
  background-color: #FFF3E0;
  color: #E65100;
  border: 2px solid #FFE0B2;
}

.penalty-badge.refusals {
  background-color: #E3F2FD;
  color: #0D47A1;
  border: 2px solid #BBDEFB;
}

.fault-icon, .refusal-icon {
  font-size: 1.8rem;
  margin-right: 8px;
}

.penalty-count {
  font-size: 1.8rem;
  font-weight: bold;
}

/* Touch-optimerade penalty-kontroller */
.penalty-controls-touch {
  display: flex;
  justify-content: space-around;
  gap: 1rem;
  background-color: #F8F9FA;
  padding: 1.5rem;
  border-radius: 16px;
  border: 1px solid #E0E0E0;
}

.penalty-group-touch {
  display: flex;
  flex-direction: column;
  align-items: center;
  flex: 1;
}

.penalty-header-touch {
  display: flex;
  flex-direction: column;
  align-items: center;
  margin-bottom: 1rem;
}

.fault-icon-large, .refusal-icon-large, .disqualify-icon-large {
  font-size: 3rem;
  margin-bottom: 0.5rem;
}

.penalty-label-large {
  font-weight: bold;
  font-size: 1.2rem;
  color: #424242;
}

.penalty-buttons-touch {
  display: flex;
  align-items: center;
  gap: 1rem;
}

.penalty-button-touch {
  width: 4rem;
  height: 4rem;
  font-size: 2rem;
  font-weight: bold;
  border: 2px solid;
  border-radius: 50%;
  cursor: pointer;
  transition: all 0.2s ease;
  display: flex;
  align-items: center;
  justify-content: center;
  min-width: 44px;
  min-height: 44px;
}

.penalty-button-touch.minus {
  background-color: #FFEBEE;
  color: #C62828;
  border-color: #FFCDD2;
}

.penalty-button-touch.plus {
  background-color: #E8F5E9;
  color: #2E7D32;
  border-color: #C8E6C9;
}

.penalty-button-touch:hover:not(:disabled) {
  transform: scale(1.05);
}

.penalty-button-touch.minus:hover:not(:disabled) {
  background-color: #FFCDD2;
}

.penalty-button-touch.plus:hover:not(:disabled) {
  background-color: #C8E6C9;
}

.penalty-button-touch:disabled {
  opacity: 0.3;
  cursor: not-allowed;
}

.penalty-value-large {
  width: 3rem;
  text-align: center;
  font-size: 2.5rem;
  font-weight: bold;
  color: #333;
}

.button-symbol {
  font-size: 2.5rem;
  line-height: 1;
}

.eliminate-button-touch {
  width: 100%;
  max-width: 200px;
  height: 4rem;
  background-color: #FFFFFF;
  color: #424242;
  border: 2px solid #E0E0E0;
  font-weight: bold;
  font-size: 1.1rem;
  border-radius: 8px;
  cursor: pointer;
  transition: all 0.2s ease;
  min-height: 44px;
}

.eliminate-button-touch:hover {
  background-color: #f1e2e2;
  transform: translateY(-2px);
}

.eliminate-button-touch.eliminated {
  background-color: #FFEBEE;
  color: #C62828;
  border-color: #FFCDD2;
}

.eliminate-button-touch.eliminated:hover {
  background-color: #FFCDD2;
}

/* Kompakt kontrollpanel */
.control-panel-compact {
  display: flex;
  justify-content: center;
}

.reset-button-compact {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 0.5rem;
  padding: 0.8rem 2rem;
  background-color: #F44336;
  color: white;
  border: none;
  border-radius: 8px;
  cursor: pointer;
  transition: all 0.2s ease;
  font-weight: bold;
  font-size: 1rem;
  min-height: 44px;
}

.reset-button-compact:hover {
  background-color: #D32F2F;
  transform: translateY(-2px);
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
}

.button-icon-small {
  font-size: 1.2rem;
}

.button-text-small {
  font-size: 1rem;
}

/* Logg-seksjon */
.log-section h3 {
  font-size: 1.5rem;
  margin: 0 0 1rem 0;
  color: #424242;
  text-align: center;
}

.log-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 1rem;
  flex-wrap: wrap;
  gap: 0.5rem;
}

.log-header h3 {
  margin: 0;
  text-align: left;
  flex: 1;
}

.log-controls {
  display: flex;
  gap: 0.5rem;
}

.log-button {
  display: flex;
  align-items: center;
  gap: 0.3rem;
  padding: 0.4rem 0.8rem;
  border: 1px solid #ddd;
  border-radius: 6px;
  background-color: white;
  color: #666;
  cursor: pointer;
  transition: all 0.2s ease;
  font-size: 0.8rem;
  min-height: 36px;
}

.log-button:hover:not(:disabled) {
  background-color: #f5f5f5;
  border-color: #ccc;
}

.log-button:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

.log-button.clear-all {
  color: #d32f2f;
  border-color: #ffcdd2;
}

.log-button.clear-all:hover:not(:disabled) {
  background-color: #ffebee;
  border-color: #d32f2f;
}

.times-list-compact {
  flex: 1;
  overflow-y: auto;
  min-height: 0;
}

.time-entry-compact {
  display: flex;
  align-items: center;
  padding: 1rem;
  border-bottom: 1px solid #EEEEEE;
  transition: background-color 0.2s;
  position: relative;
}

.time-entry-compact:hover {
  background-color: #FAFAFA;
}

.time-entry-compact:hover .remove-button {
  opacity: 1;
}

.remove-button {
  position: absolute;
  right: 0.5rem;
  top: 50%;
  transform: translateY(-50%);
  width: 24px;
  height: 24px;
  border: none;
  border-radius: 50%;
  background-color: #ffebee;
  color: #d32f2f;
  cursor: pointer;
  transition: all 0.2s ease;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 16px;
  opacity: 0;
  min-width: 32px;
  min-height: 32px;
}

.remove-button:hover {
  background-color: #ffcdd2;
  transform: translateY(-50%) scale(1.1);
}

/* På touch-enheter, vis remove-knappen alltid */
@media (hover: none) {
  .remove-button {
    opacity: 0.7;
  }
}

.time-number-large {
  width: 3rem;
  height: 3rem;
  font-size: 1.2rem;
  font-weight: bold;
  background-color: #F5F5F5;
  color: #424242;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  margin-right: 1rem;
  flex-shrink: 0;
}

.time-details-compact {
  flex: 1;
  display: flex;
  flex-direction: column;
  gap: 0.2rem;
}

.time-main-info {
  display: flex;
  justify-content: space-between;
  align-items: baseline;
}

.time-of-day {
  font-size: 0.9rem;
  color: #666;
}

.time-value-large {
  font-family: 'Roboto Mono', monospace;
  font-size: 1.4rem;
  font-weight: bold;
  color: #333;
}

.simple-delete {
  font-size: 1rem;
  cursor: pointer;
  padding: 0.3rem 0.5rem;
  opacity: 0.5;
  transition: all 0.2s ease;
  border-radius: 4px;
  background-color: transparent;
  color: #d32f2f;
  min-width: 32px;
  min-height: 32px;
  display: flex;
  align-items: center;
  justify-content: center;
}

.simple-delete:hover {
  opacity: 1;
  background-color: #ffebee;
  transform: scale(1.05);
}

/* På touch-enheter, vis delete-knappen tydligere */
@media (hover: none) {
  .simple-delete {
    opacity: 0.7;
  }
}

.time-penalties-row {
  display: flex;
  justify-content: space-between;
  align-items: center;
  width: 100%;
}

.time-penalties {
  display: flex;
  gap: 0.8rem;
  align-items: center;
  flex-wrap: wrap;
  justify-content: flex-end;
}

.penalty-item {
  display: flex;
  align-items: center;
  gap: 4px;
  font-size: 0.9rem;
  background-color: #F5F5F5;
  padding: 3px 8px;
  border-radius: 6px;
}

.clean-run {
  font-size: 0.9rem;
  color: #4CAF50;
  font-weight: 500;
  font-style: italic;
}

.penalty-item {
  display: flex;
  align-items: center;
  gap: 4px;
  font-size: 0.9rem;
  background-color: #F5F5F5;
  padding: 3px 8px;
  border-radius: 6px;
}

.fault-icon.small, .refusal-icon.small {
  font-size: 1rem;
}

.eliminated-indicator {
  background-color: #FFEBEE;
  color: #C62828;
  padding: 3px 8px;
  border-radius: 6px;
  font-weight: bold;
  font-size: 0.9rem;
}

.no-times {
  text-align: center;
  color: #757575;
  padding: 2rem;
  font-style: italic;
}

/* Responsiv design för iPad */
@media (max-width: 1024px) {
  .main-content {
    flex-direction: column;
    gap: 1rem;
  }
  
  .timer-section {
    flex: none;
  }
  
  .log-section {
    flex: none;
    max-height: 300px;
  }
  
  .timer-display .time {
    font-size: 4.5rem;
  }
  
  .penalty-controls-touch {
    flex-direction: column;
    gap: 1.5rem;
  }
}

@media (max-width: 768px) {
  .timer-display .time {
    font-size: 3.5rem;
  }
  
  .penalty-button-touch {
    width: 3.5rem;
    height: 3.5rem;
    font-size: 1.8rem;
  }
  
  .penalty-value-large {
    font-size: 2rem;
  }
}
</style>