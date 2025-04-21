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
    }
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
        // Endrer fra toLocaleTimeString() til formatering som matcher systemets tid
        const date = new Date(lastTime);
        return Intl.DateTimeFormat("NO", {
        hour: "numeric",
        minute: "numeric",
        second: "numeric"
        }).format(date);
    }
    return "Ingen";
    },
    timeElapsedSinceLastBreak() {
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
        const now = Date.now();
        const elapsedMs = now - lastTime;
        const elapsedSeconds = Math.floor(elapsedMs / 1000);
        
        if (elapsedSeconds < 60) {
        return `${elapsedSeconds} sek siden`;
        } else if (elapsedSeconds < 3600) {
        const minutes = Math.floor(elapsedSeconds / 60);
        return `${minutes} min siden`;
        } else {
        const hours = Math.floor(elapsedSeconds / 3600);
        return `${hours} time${hours > 1 ? 'r' : ''} siden`;
        }
    }
    
    return "Ingen passering registrert";
    }

  }
};
</script>

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
        <!--<span class="elapsed-time">({{ timeElapsedSinceLastBreak }})</span>-->
        </div>
      </div>
      
      <!-- Hurtighandlinger -->
      <div class="dashboard-panel actions-panel">
        <h2>Hurtighandlinger</h2>
        <div class="action-buttons">
          <button class="action-button sync">Synkroniser tid</button>
          <button class="action-button calibrate">Kalibrer sensorer</button>
          <button class="action-button reset">Nullstill timer</button>
        </div>
      </div>
    </div>
  </div>
</template>

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

.action-button:hover {
  opacity: 0.9;
}

.elapsed-time {
  font-size: 0.85em;
  color: #777;
  margin-left: 8px;
  font-style: italic;
}
</style>