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
    }
  },
  data() {
    return {
      currentView: "dashboard" // Standard visning
    };
  },
  methods: {
    navigateTo(view) {
      this.currentView = view;
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
      </nav>
    </header>
    
    <!-- Hovedinnhold basert på valgt visning -->
    <main class="app-content">
      <dashboard v-if="currentView === 'dashboard'" :time="time" :poles="poles" :breaks="breaks" />
      <timer-view v-else-if="currentView === 'timer'" :time="time" :poles="poles" :breaks="breaks" />
      <devices-view v-else-if="currentView === 'devices'" :poles="poles" />
      <config-view v-else-if="currentView === 'config'" />
      <log-view v-else-if="currentView === 'log'" :breaks="breaks" :passages="passages" />
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

.app-content {
  flex: 1;
  padding: 24px;
  background-color: #f5f5f5;
  overflow-y: auto;
}
</style>