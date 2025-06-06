<template>
  <div>
    <h1>Enheter</h1>
    
    <div class="devices-container">
      <!-- System-info øverst -->
      <div class="system-info-panel">
        <div class="system-info-content">
          <div class="system-id-display">
            <span class="system-label">System ID:</span>
            <span class="system-id-value">{{ formatSystemId(systemId) }}</span>
          </div>
          <div class="discovery-status">
            <span class="discovery-label">Discovery:</span>
            <span class="discovery-indicator" :class="{ active: discoveryActive }">
              {{ discoveryActive ? 'Aktiv' : 'Inaktiv' }}
            </span>
          </div>
        </div>
      </div>

      <!-- Status-sammendrag -->
      <div class="status-summary">
        <div class="status-card">
          <div class="status-value">{{ (pairedPolesInternal || []).length }}</div>
          <div class="status-label">Tilknyttede målestolper</div>
        </div>
        <div class="status-card">
          <div class="status-value">{{ totalDiscoveredPoles }}</div>
          <div class="status-label">Nye målestolper funnet</div>
        </div>
        <div class="status-card">
          <div class="status-value">{{ countActiveSensors() }}</div>
          <div class="status-label">Aktive sensorer</div>
        </div>
        <button @click="refreshAllData" class="scan-button" :disabled="isScanning">
          <span class="icon">🔍</span>
          {{ isScanning ? 'Søker...' : 'Oppdater liste' }}
        </button>
      </div>

      <!-- Nye målestolper som søker tilknytning -->
      <div class="unassigned-poles" v-if="discoveredPolesFiltered.length > 0">
        <h2>🆕 Nye målestolper ({{ discoveredPolesFiltered.length }})</h2>
        
        <div class="poles-grid">
          <div v-for="pole in discoveredPolesFiltered" :key="pole.mac" class="pole-card unassigned">

            <div class="pole-header">
              <div class="pole-title-section">
                <div class="pole-title">
                  <h3>{{ pole.name || 'Navnløs målestolpe' }}</h3>
                  <div class="pole-subtitle">
                    <span class="mac-address">{{ formatMac(pole.mac) }}</span>
                    <span class="firmware-version" v-if="pole.firmware_version">
                      v{{ pole.firmware_version }}
                    </span>
                  </div>
                </div>
                
                <div class="pole-indicators">                 
                  <div class="pole-status-badge" :class="getConnectionStatus(pole)">
                    <span class="status-dot"></span>
                    {{ getConnectionText(pole) }}
                  </div>

                  <!-- NY SIGNALKVALITET-INDIKATOR -->
                  <div class="signal-quality" v-if="getSignalQuality(pole)">
                    <div class="signal-primary" :class="getSignalQualityClass(getSignalQuality(pole).quality)">
                      <span class="signal-icon">{{ getSignalIcon(getSignalQuality(pole).rssi) }}</span>
                      <span class="signal-value">{{ getSignalQuality(pole).quality }}%</span>
                    </div>
                    <div class="signal-text">
                      {{ getSignalText(getSignalQuality(pole).rssi, getSignalQuality(pole).quality) }}
                    </div>
                  </div>
                  
                  <!-- Fallback for manglende signaldata -->
                  <div class="signal-quality" v-else>
                    <div class="signal-primary signal-unknown">
                      <span class="signal-icon">📶</span>
                      <span class="signal-value">--</span>
                    </div>
                    <div class="signal-text">Ukjent signal</div>
                  </div>

                </div>
              </div>
              
              <div class="pole-actions-compact">
                <button @click="identifyPole(pole)" class="icon-button identify-btn" title="Identifiser">
                  <span class="icon">🔍</span>
                </button>
                <button @click="expandDevice(pole)" class="icon-button expand-btn" v-if="!isExpanded(pole)" title="Vis detaljer">
                  <span class="icon">⬇️</span>
                </button>
                <button @click="collapseDevice(pole)" class="icon-button collapse-btn" v-else title="Skjul detaljer">
                  <span class="icon">⬆️</span>
                </button>
              </div>
            </div>


            
            <div class="pole-info">
              <div class="info-row">
                <div class="info-label">MAC-adresse:</div>
                <div class="info-value">{{ formatMac(pole.mac) }}</div>
              </div>
              <div class="info-row">
                <div class="info-label">Avstand:</div>
                <div class="info-value">~{{ pole.estimated_distance || 'Ukjent' }}m</div>
              </div>
              <div class="info-row">
                <div class="info-label">Signalstyrke:</div>
                <div class="info-value">{{ pole.rssi || 'Ukjent' }} dBm</div>
              </div>
              <div class="info-row">
                <div class="info-label">Sist sett:</div>
                <div class="info-value">{{ formatTimeSince(pole.last_seen) }}</div>
              </div>
            </div>
            
            <div class="pole-actions-unassigned">
              <button @click="identifyPole(pole)" :disabled="pole.identifying" class="action-button identify">
                <span v-if="pole.identifying" class="icon">🔶</span>
                <span v-else class="icon">🔍</span>
                {{ pole.identifying ? 'Blinker...' : 'Identifiser' }}
              </button>
              <button @click="assignPole(pole)" class="action-button assign">
                <span class="icon">✅</span>
                Tilknytt
              </button>
            </div>
          </div>
        </div>
        
        <div class="instructions">
          <div class="instructions-header">
            <span class="icon">💡</span>
            <strong>Bruksanvisning:</strong>
          </div>
          <ol>
            <li>Klikk <strong>"Identifiser"</strong> på en målestolpe</li>
            <li>Gå til den fysiske målestolpen som blinker oransje</li>
            <li>Kom tilbake og klikk <strong>"Tilknytt"</strong> hvis det var riktig målestolpe</li>
          </ol>
        </div>
      </div>

      <!-- Tilknyttede målestolper -->
      <div class="paired-poles" v-if="(pairedPolesInternal || []).length > 0">
        <h2>🔗 Tilknyttede målestolper ({{ (pairedPolesInternal || []).length }})</h2>
        
        <div class="poles-grid">
          <div v-for="pole in pairedPolesInternal" :key="pole.mac" class="pole-card paired">
            <div class="pole-header">
              <div class="pole-title">
                <h3>{{ pole.name || 'Navnløs målestolpe' }}</h3>
                <div class="pole-status" :class="getConnectionStatus(pole)">
                  {{ getConnectionText(pole) }}
                </div>
              </div>
              <div class="pole-actions-compact">
                <button @click="expandDevice(pole)" class="icon-button" v-if="!isExpanded(pole)">
                  <span class="icon">⬇️</span>
                </button>
                <button @click="collapseDevice(pole)" class="icon-button" v-else>
                  <span class="icon">⬆️</span>
                </button>
              </div>
            </div>
            
            <div class="pole-info">
              <div class="info-row">
                <div class="info-label">MAC-adresse:</div>
                <div class="info-value">{{ formatMac(pole.mac) }}</div>
              </div>
              <div class="info-row">
                <div class="info-label">Tilknyttet:</div>
                <div class="info-value">{{ formatTimeSince(pole.paired_time) }}</div>
              </div>
              <div class="info-row">
                <div class="info-label">Sist aktiv:</div>
                <div class="info-value">{{ formatTimeSince(pole.last_seen) }}</div>
              </div>
            </div>
            
            <div class="pole-actions-footer">
            <div class="action-group primary">
              <button @click="calibrateSensors(pole)" class="action-button calibrate" 
                      :disabled="isCalibratingMap[pole.mac]">
                <span class="icon">⚙️</span>
                <span class="text">{{ isCalibratingMap[pole.mac] ? 'Kalibrerer...' : 'Kalibrer' }}</span>
              </button>
              
              <button @click="identifyPole(pole)" class="action-button identify">
                <span class="icon">🔍</span>
                <span class="text">Identifiser</span>
              </button>
            </div>
            
            <div class="action-group secondary">
              <button @click="openRenameModal(pole)" class="action-button rename">
                <span class="icon">✏️</span>
                <span class="text">Navn</span>
              </button>
              
              <button @click="openRestartModal(pole)" class="action-button restart">
                <span class="icon">🔄</span>
                <span class="text">Restart</span>
              </button>
            </div>
            
            <div class="action-group danger">
              <button @click="openPowerModal(pole)" class="action-button power" 
                      :class="{ 'power-off': !isPowerOn(pole) }">
                <span class="icon">⚡</span>
                <span class="text">{{ isPowerOn(pole) ? 'Av' : 'På' }}</span>
              </button>
              
              <button @click="unpairPole(pole)" class="action-button unpair">
                <span class="icon">🗑️</span>
                <span class="text">Fjern</span>
              </button>
            </div>
          </div>





            <!-- Utvidet visning for paired poles -->
            <div v-if="isExpanded(pole)" class="device-expanded">
              <div class="sensor-section">
                <h3>Sensorer ({{ getSensorCount(pole) }} av 7 aktive)</h3>
                
                <!-- Finn korresponderende pole-data -->
                <div v-if="getCorrespondingPoleData(pole)" class="sensor-grid">
                  <div v-for="(value, index) in getCorrespondingPoleData(pole).values" 
                       :key="index" 
                       class="sensor-item"
                       :class="{ active: isSensorActive(value), enabled: isSensorEnabled(pole, index) }">
                    
                    <div class="sensor-header">
                      <span class="sensor-label">Sensor {{ index + 1 }}</span>
                      <span class="sensor-status" :class="getSensorStatusClass(value)">
                        {{ getSensorStatusText(value) }}
                      </span>
                    </div>
                    
                    <div class="sensor-value">{{ value || 0 }}</div>
                    
                    <div class="sensor-bar-container">
                      <div class="sensor-bar" 
                           :style="{ width: getSensorBarWidth(value) }"
                           :class="getSensorBarColor(value)">
                      </div>
                      
                      <!-- Break threshold indicator -->
                      <div v-if="getBreakThreshold(pole, index)" 
                           class="sensor-threshold" 
                           :style="{ left: getSensorThresholdPosition(pole, index) }">
                      </div>
                    </div>
                    
                    <div class="sensor-info">
                      <span class="threshold-text">
                        Terskel: {{ getBreakThreshold(pole, index) || 'N/A' }}
                      </span>
                      <span class="enabled-text" :class="{ disabled: !isSensorEnabled(pole, index) }">
                        {{ isSensorEnabled(pole, index) ? 'Aktivert' : 'Deaktivert' }}
                      </span>
                    </div>
                  </div>
                </div>
                
                <div v-else class="no-sensor-data">
                  <span class="icon">⚠️</span>
                  <p>Ingen sensordata tilgjengelig for denne målestolpen</p>
                  <small>Målestolpen må være tilkoblet via TCP for å vise sensordata</small>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
      
      <!-- Ferdig oppsatt system eller ingen enheter -->
      <div v-if="(pairedPolesInternal || []).length === 0 && (discoveredPolesInternal || []).length === 0" class="no-devices">
        <div class="empty-state">
          <div class="empty-icon">📡</div>
          <h3>Ingen målestolper funnet</h3>
          <p v-if="!discoveryActive">Discovery er ikke aktiv. Systemet leter ikke etter nye målestolper.</p>
          <p v-else>Systemet leter etter målestolper. Sørg for at målestolpene er påslått og innenfor rekkevidde.</p>
        </div>
      </div>
    </div>

    <!-- Restart Modal -->
    <div v-if="showRestartModal" class="modal-overlay" @click="cancelRestart">
      <div class="modal-content" @click.stop>
        <div class="modal-header">
          <h2>Restart {{ selectedPole?.name }}</h2>
          <button @click="cancelRestart" class="close-button">×</button>
        </div>
        <div class="modal-body">
          <div class="restart-options">
            <div class="restart-option">
              <input type="radio" id="restart1" value="1" v-model="restartType">
              <label for="restart1">
                <strong>Soft Restart</strong>
                <p>Vanlig restart - bevarer alle innstillinger</p>
              </label>
            </div>
            <div class="restart-option">
              <input type="radio" id="restart2" value="2" v-model="restartType">
              <label for="restart2">
                <strong>Hard Restart</strong>
                <p>Tvungen restart - nullstiller midlertidige data</p>
              </label>
            </div>
            <div class="restart-option">
              <input type="radio" id="restart3" value="3" v-model="restartType">
              <label for="restart3">
                <strong>Factory Reset</strong>
                <p>Sletter systemtilknytning - krever ny pairing</p>
              </label>
            </div>
          </div>
        </div>
        <div class="modal-footer">
          <button @click="cancelRestart" class="cancel-button">Avbryt</button>
          <button @click="confirmRestart" :disabled="isRestarting" class="confirm-button restart">
            {{ isRestarting ? 'Restarter...' : 'Restart' }}
          </button>
        </div>
      </div>
    </div>

    <!-- Power Modal -->
    <div v-if="showPowerModal" class="modal-overlay" @click="cancelPowerOff">
      <div class="modal-content" @click.stop>
        <div class="modal-header">
          <h2>Slå av {{ selectedPole?.name }}</h2>
          <button @click="cancelPowerOff" class="close-button">×</button>
        </div>
        <div class="modal-body">
          <p>Er du sikker på at du vil slå av denne enheten?</p>
          <div class="power-off-info">
            <h4>Viktig informasjon:</h4>
            <ul>
              <li>Enheten går i deep sleep-modus</li>
              <li>All kommunikasjon stoppes</li>
              <li>Enheten må startes manuelt med reset-knappen</li>
            </ul>
          </div>
        </div>
        <div class="modal-footer">
          <button @click="cancelPowerOff" class="cancel-button">Avbryt</button>
          <button @click="confirmPowerOff" :disabled="isPoweringOff" class="confirm-button power-off">
            {{ isPoweringOff ? 'Slår av...' : 'Slå av' }}
          </button>
        </div>
      </div>
    </div>

    <!-- Rename Modal -->
    <div v-if="showRenameModal" class="modal-overlay" @click="cancelRename">
      <div class="modal-content" @click.stop>
        <div class="modal-header">
          <h2>Endre navn</h2>
          <button @click="cancelRename" class="close-button">×</button>
        </div>
        <div class="modal-body">
          <div class="form-group">
            <label for="device-name">Nytt navn:</label>
            <input type="text" id="device-name" v-model="newDeviceName" class="text-input" placeholder="Skriv inn nytt navn">
          </div>
        </div>
        <div class="modal-footer">
          <button @click="cancelRename" class="cancel-button">Avbryt</button>
          <button @click="confirmRename" class="confirm-button">Lagre</button>
        </div>
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
    },
    // FIKSET: Flyttet disse inn i props-objektet
    discoveredPoles: {
      type: Array,
      default: () => []
    },
    pairedPoles: {
      type: Array,
      default: () => []
    },
    systemId: {
      type: String,
      default: ""
    },
    currentView: {
      type: String,
      default: ''
   }
  },
  data() {
    console.log('DevicesView data() kjører');
    return {
      discoveredPolesInternal: [],
      pairedPolesInternal: [],


    // Signal quality tracking
    signalData: {},
    signalUpdateInterval: null,



      // Expanded state
      expandedPoles: {},
      // Discovery state
      discoveryActive: false,
      isScanning: false,
      
      // Discovered poles state
      discoveredPolesInternal: [],
      pairedPolesInternal: [],

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

      currentRoute: this.$route?.path || ''      

    };
  },

  computed: {
  // Kombinerte tilkoblede enheter (både paired og gamle poles)
  connectedPoles() {
    console.log('connectedPoles kjører, pairedPolesInternal:', this.pairedPolesInternal);
      // Kombiner paired poles med eksisterende poles for bakoverkompatibilitet
      const combined = [...(this.pairedPolesInternal || [])];
      
      // Legg til gamle poles som ikke finnes i paired
      this.poles.forEach(pole => {
        const existsInPaired = this.pairedPolesInternal.some(paired => 
          paired.mac === pole.mac
        );
        if (!existsInPaired && pole.values && Array.isArray(pole.values) && pole.values.some(v => v > 0)) {
          combined.push({
            ...pole,
            name: pole.name || `Pole ${pole.id}`,
            paired: false
          });
        }
      });
      
      return combined;
    },
    
    // Nye computed properties for discovery
    discoveredPolesFiltered() {
      console.log('discoveredPolesFiltered kjører, discoveredPolesInternal:', this.discoveredPolesInternal);
      if (!Array.isArray(this.discoveredPolesInternal)) {
         return [];
      }
      return this.discoveredPolesInternal.filter(pole => {
        // Vis kun poles som ikke allerede er paired
        return !(this.pairedPolesInternal || []).some(paired => {
          if (!paired || !paired.mac || !pole || !pole.mac) return false;
          return this.macAddressesEqual(paired.mac, pole.mac);
        });
      });
    },
    
    totalDiscoveredPoles() {
      return this.discoveredPolesFiltered ? this.discoveredPolesFiltered.length : 0;
    },

  },




  methods: {

    // Finn korresponderende pole-data fra poles prop
    getCorrespondingPoleData(pairedPole) {
      return this.poles.find(pole => 
        pole.mac === pairedPole.mac || 
        this.macAddressesEqual(pole.mac, pairedPole.mac)
      );
    },

    getSensorCount(pole) {
      const poleData = this.getCorrespondingPoleData(pole);
      if (!poleData || !poleData.values) return 0;
      return poleData.values.filter(v => this.isSensorActive(v)).length;
    },

    isSensorActive(value) {
      return value && value > 100; // Juster terskel etter behov
    },

    isSensorEnabled(pole, index) {
      const poleData = this.getCorrespondingPoleData(pole);
      return poleData?.enabled?.[index] !== false;
    },

    getSensorStatusClass(value) {
      if (!value || value < 100) return 'inactive';
      if (value > 3000) return 'high';
      if (value > 2000) return 'medium';
      return 'low';
    },

    getSensorStatusText(value) {
      if (!value || value < 100) return 'Inaktiv';
      if (value > 3000) return 'Høy';
      if (value > 2000) return 'Medium';
      return 'Lav';
    },

    getSensorBarWidth(value) {
      if (!value) return '0%';
      const percent = Math.min(100, Math.max(0, (value / 4096) * 100));
      return `${percent}%`;
    },

    getSensorBarColor(value) {
      if (!value || value < 100) return 'bar-inactive';
      if (value > 3000) return 'bar-high';
      if (value > 2000) return 'bar-medium';
      return 'bar-low';
    },

    getBreakThreshold(pole, index) {
      const poleData = this.getCorrespondingPoleData(pole);
      return poleData?.br_limit?.[index];
    },

    getSensorThresholdPosition(pole, index) {
      const threshold = this.getBreakThreshold(pole, index);
      if (!threshold) return '50%';
      const percent = Math.min(100, Math.max(0, (threshold / 4096) * 100));
      return `${percent}%`;
    },



    activated() {
      console.log('DevicesView aktivert - laster data automatisk');
      this.refreshAllData();
    },


    async refreshAllData() {
      console.log('🔄 Refreshing all device data...');
      await this.loadSystemInfo();
      await this.loadDiscoveredPoles();
      await this.loadPairedPoles();
      console.log('✅ All data refreshed');
    },


    // NYE metoder for ekte signalstyrke:
    
    /**
     * Hent signalkvalitet for en målestolpe
     */
     getSignalQuality(pole) {
      if (!pole || !pole.mac) return null;
      return this.signalData[pole.mac] || null;
    },

    /**
     * Få signal-ikon basert på RSSI
     */
    getSignalIcon(rssi) {
      if (rssi >= -40) return '📶'; // 4 streker
      if (rssi >= -50) return '📶'; // 3 streker
      if (rssi >= -60) return '📶'; // 2 streker  
      if (rssi >= -70) return '📶'; // 1 strek
      return '📶'; // Ingen signal
    },

    /**
     * Få signal CSS-klasse basert på kvalitet
     */
    getSignalQualityClass(quality) {
      if (quality >= 80) return 'signal-excellent';
      if (quality >= 60) return 'signal-good';
      if (quality >= 40) return 'signal-fair';
      if (quality >= 20) return 'signal-poor';
      return 'signal-none';
    },

    /**
     * Få signaltekst basert på RSSI
     */
    getSignalText(rssi, quality) {
      if (quality >= 80) return `Utmerket (${rssi} dBm)`;
      if (quality >= 60) return `Meget god (${rssi} dBm)`;
      if (quality >= 40) return `God (${rssi} dBm)`;
      if (quality >= 20) return `Dårlig (${rssi} dBm)`;
      return `Meget dårlig (${rssi} dBm)`;
    },

    /**
     * Last signalkvalitet fra API
     */
     async loadSignalQuality() {
      try {
        console.log('🔄 Starting loadSignalQuality...');
        console.log('📍 serverAddress:', this.serverAddress);
        
        const url = `http://${this.serverAddress}/api/v1/signal/quality`;
        console.log('📡 Calling URL:', url);
        
        const response = await fetch(url);
        console.log('📡 Response received:', response.status, response.ok);
        
        const data = await response.json();
        console.log('📊 Raw signal API data:', data);
        
        if (data.status === 'success') {
          console.log('✅ API status success');
          console.log('📋 data.signal_data:', data.signal_data);
          console.log('📋 data.signal_data length:', data.signal_data ? data.signal_data.length : 'undefined');
          
          // Konverter array til objekt indeksert etter MAC
          const signalMap = {};
          data.signal_data.forEach(signal => {
            signalMap[signal.mac] = signal;
          });
          
          console.log('🔧 BEFORE assignment:');
          console.log('  - this.signalData:', this.signalData);
          
          this.signalData = signalMap;
          
          console.log('✅ AFTER assignment:');
          console.log('  - this.signalData:', this.signalData);
          console.log('  - Object.keys(signalMap):', Object.keys(signalMap));
          
          console.log('📶 Signalkvalitet oppdatert for', Object.keys(signalMap).length, 'målestolper');
          
          // Force Vue reactivity update
          this.$forceUpdate && this.$forceUpdate();
          
        } else {
          console.error('❌ API status not success:', data.status);
        }
      } catch (error) {
        console.error('💥 Feil ved lasting av signalkvalitet:', error);
        console.error('💥 Error stack:', error.stack);
      }
    },



    // Hjelpemetoder for MAC-adresse håndtering
  macAddressesEqual(mac1, mac2) {
    if (!mac1 || !mac2) return false;
    
    // Normaliser MAC-adresser (fjern : og gjør lowercase)
    const normalize = (mac) => {
      if (Array.isArray(mac)) {
        // Konverter array til string format
        return mac.map(b => b.toString(16).padStart(2, '0')).join('').toLowerCase();
      }
      return mac.replace(/:/g, '').toLowerCase();
    };
    
    return normalize(mac1) === normalize(mac2);
  },
  
  formatMac(mac) {
    if (Array.isArray(mac)) {
      return mac.map(b => b.toString(16).padStart(2, '0')).join(':');
    }
    return mac;
  },
  
  formatSystemId(systemId) {
    if (!systemId) return 'Ukjent';
    return systemId.toUpperCase();
  },


  formatTimeSince(timestamp) {
    if (!timestamp) return 'Ukjent';
    
    const now = Date.now() / 1000;
    const diff = now - timestamp;
    
    if (diff < 60) return 'Nettopp';
    if (diff < 3600) return `${Math.floor(diff / 60)} min siden`;
    if (diff < 86400) return `${Math.floor(diff / 3600)} timer siden`;
    return `${Math.floor(diff / 86400)} dager siden`;
  },
  


  getConnectionStatus(pole) {
    if (!pole.last_seen) return 'unknown';
    
    const now = Date.now() / 1000;
    const timeSince = now - pole.last_seen;
    
    if (timeSince < 300) return 'connected';      // Mindre enn 5 min
    if (timeSince < 3600) return 'idle';          // Mindre enn 1 time
    return 'disconnected';                        // Mer enn 1 time
  },
  
  getConnectionText(pole) {
    const status = this.getConnectionStatus(pole);
    switch (status) {
      case 'connected': return 'Tilkoblet';
      case 'idle': return 'Inaktiv';
      case 'disconnected': return 'Frakoblet';
      default: return 'Ukjent';
    }
  },


    countActiveSensors() {
      let count = 0;
      // Bruk connectedPoles i stedet for poles
      this.connectedPoles.forEach(pole => {
        count += this.countPoleActiveSensors(pole);
      });
      return count;
    },

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
    

    // API-metoder for discovery og pairing
    async loadDiscoveredPoles() {
      try {
        const response = await fetch(`http://${this.serverAddress}/api/v1/poles/discovered`);
        const data = await response.json();
        
        if (data.status === 'success') {
          this.discoveredPolesInternal = data.poles || [];
          this.discoveryActive = data.discovery_active || false;
          console.log('Lastet discovered poles:', this.discoveredPolesInternal.length);
        }
      } catch (error) {
        console.error('Feil ved lasting av discovered poles:', error);
        this.showToast('error', 'Kunne ikke laste oppdagede målestolper');
      }
    },
    
// FIKSET loadPairedPoles() metode i DevicesView.vue
// Erstatt den eksisterende loadPairedPoles() funksjonen med denne:

async loadPairedPoles() {
  try {
    console.log('🔄 Starting loadPairedPoles...');
    console.log('📍 serverAddress:', this.serverAddress);
    console.log('📍 Computed apiBaseUrl:', this.apiBaseUrl);
    
    // Use the same URL computation as other methods
    const url = `http://${this.serverAddress}/api/v1/poles/paired`;
    console.log('📡 Calling URL:', url);
    
    const response = await fetch(url);
    console.log('📡 Response received:', response.status, response.ok);
    
    const data = await response.json();
    console.log('📊 Raw API data:', data);
    
    if (data.status === 'success') {
      console.log('✅ API status success');
      console.log('📋 data.poles:', data.poles);
      console.log('📋 data.poles length:', data.poles ? data.poles.length : 'undefined');
      console.log('📋 data.poles type:', typeof data.poles, Array.isArray(data.poles));
      
      // CRITICAL: Log before assignment
      console.log('🔧 BEFORE assignment:');
      console.log('  - this.pairedPolesInternal:', this.pairedPolesInternal);
      console.log('  - this.pairedPolesInternal length:', this.pairedPolesInternal.length);
      console.log('  - this exists:', !!this);
      console.log('  - this.pairedPolesInternal exists:', !!this.pairedPolesInternal);
      
      // Assignment with extra safety
      const newPoles = data.poles || [];
      console.log('🔧 Assigning newPoles:', newPoles);
      
      this.pairedPolesInternal = newPoles;
      
      // CRITICAL: Verify assignment worked
      console.log('✅ AFTER assignment:');
      console.log('  - this.pairedPolesInternal:', this.pairedPolesInternal);
      console.log('  - this.pairedPolesInternal length:', this.pairedPolesInternal.length);
      console.log('  - Assignment successful:', this.pairedPolesInternal === newPoles);
      
      console.log('💾 Lastet paired poles:', this.pairedPolesInternal.length);
      
      // Force Vue reactivity update
      this.$forceUpdate && this.$forceUpdate();
      
    } else {
      console.error('❌ API status not success:', data.status);
      this.showToast && this.showToast('error', `API feil: ${data.message || 'Ukjent feil'}`);
    }
  } catch (error) {
    console.error('💥 Feil ved lasting av paired poles:', error);
    console.error('💥 Error stack:', error.stack);
    this.showToast && this.showToast('error', 'Kunne ikke laste tilknyttede målestolper');
  }
},
    
    async loadSystemInfo() {
      try {
        const response = await fetch(`http://${this.serverAddress}/api/v1/system/id`);
        const data = await response.json();
        
        if (data.status === 'success') {
          this.systemId = data.system_id || '';
          console.log('System ID:', this.systemId);
        }
      } catch (error) {
        console.error('Feil ved lasting av system-info:', error);
      }
    },


// Pairing og administrasjon
async assignPole(pole) {
      try {
        this.showToast('info', `Tilknytter ${pole.device_name}...`);
        
        const response = await fetch(`http://${this.serverAddress}/api/v1/poles/assign`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json'
          },
          body: JSON.stringify({
            mac: this.formatMac(pole.mac)
          })
        });
        
        const data = await response.json();
        
        if (data.status === 'success') {
          this.showToast('success', `${pole.device_name} tilknyttet systemet`);
          
          // Oppdater lister
          await this.loadDiscoveredPoles();
          await this.loadPairedPoles();
        } else {
          this.showToast('error', `Kunne ikke tilknytte målestolpe: ${data.message || 'Ukjent feil'}`);
        }
      } catch (error) {
        console.error('Feil ved tilknytning av målestolpe:', error);
        this.showToast('error', 'Feil ved kommunikasjon med serveren');
      }
    },
    
    async unpairPole(pole) {
      if (!confirm(`Er du sikker på at du vil fjerne "${pole.name}" fra systemet?`)) {
        return;
      }
      
      try {
        this.showToast('info', `Fjerner ${pole.name}...`);
        
        const response = await fetch(`http://${this.serverAddress}/api/v1/poles/unpair`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json'
          },
          body: JSON.stringify({
            mac: this.formatMac(pole.mac)
          })
        });
        
        const data = await response.json();
        
        if (data.status === 'success') {
          this.showToast('success', `${pole.name} fjernet fra systemet`);
          await this.loadPairedPoles();
        } else {
          this.showToast('error', `Kunne ikke fjerne målestolpe: ${data.message || 'Ukjent feil'}`);
        }
      } catch (error) {
        console.error('Feil ved fjerning av målestolpe:', error);
        this.showToast('error', 'Feil ved kommunikasjon med serveren');
      }
    },





    async identifyPole(pole) {
      try {
        this.showToast('info', `Sender blinksignal til ${pole.device_name || pole.name}...`);
        
        const response = await fetch(`http://${this.serverAddress}/api/v1/poles/identify`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json'
          },
          body: JSON.stringify({
            mac: this.formatMac(pole.mac),
            duration: 10  // 10 sekunder blinking
          })
        });
        
        const data = await response.json();
        
        if (data.status === 'success') {
          this.showToast('success', `${pole.device_name || pole.name} blinker nå for identifikasjon`);
          
          // Oppdater identifying status lokalt hvis det er en discovered pole
          if (pole.device_name) {
            pole.identifying = true;
            setTimeout(() => {
              pole.identifying = false;
            }, 10000); // Tilbakestill etter 10 sekunder
          }
        } else {
          this.showToast('error', `Kunne ikke sende blinksignal: ${data.message || 'Ukjent feil'}`);
        }
      } catch (error) {
        console.error('Feil ved identifisering av målestolpe:', error);
        this.showToast('error', 'Feil ved kommunikasjon med serveren');
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
      console.log('DevicesView mounted');
      console.log('discoveredPoles prop:', this.discoveredPoles);
      console.log('pairedPoles prop:', this.pairedPoles);
      console.log('discoveredPolesInternal:', this.discoveredPolesInternal);
      console.log('pairedPolesInternal:', this.pairedPolesInternal);

      
      // Last inn initial data
      this.refreshAllData();  

      // Start periodisk oppdatering av signalkvalitet
      this.loadSignalQuality();
      this.signalUpdateInterval = setInterval(() => {
        this.loadSignalQuality();
      }, 5000); // Oppdater hvert 5. sekund


      
      // Initialiser strømstatus for alle målestolper (eksisterende kode)
      this.poles.forEach(pole => {
        if (!(pole.mac in this.powerStatus)) {
          this.powerStatus[pole.mac] = true; // Anta at alle er på ved oppstart
        }
      });
      
      // Sett opp periodisk oppdatering av discovered poles
      this.discoveryInterval = setInterval(() => {
        if (this.discoveryActive) {
          this.loadDiscoveredPoles();
        }
      }, 5000); // Oppdater hvert 5. sekund
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
      },

      currentView: {
        handler(newView, oldView) {
          console.log('currentView endret fra:', oldView, 'til:', newView);
          if (newView === 'devices') {
            console.log('Navigert til enheter-siden - refresher data automatisk');
            this.$nextTick(() => {
              this.refreshAllData();
            });
          }
        },
        immediate: false
      }

    }
  },
  // Lifecycle hooks
  beforeDestroy() {
    if (this.discoveryInterval) {
      clearInterval(this.discoveryInterval);
    }
    if (this.signalUpdateInterval) {
      clearInterval(this.signalUpdateInterval);
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




/* NYE signal-quality stiler */
.signal-quality {
  display: flex;
  flex-direction: column;
  align-items: center;
  margin-left: 12px;
}

.signal-primary {
  display: flex;
  align-items: center;
  gap: 4px;
  padding: 4px 8px;
  border-radius: 12px;
  font-weight: 600;
  font-size: 12px;
  transition: all 0.3s ease;
}

.signal-icon {
  font-size: 14px;
}

.signal-value {
  font-size: 11px;
  font-weight: 700;
}

.signal-text {
  font-size: 9px;
  margin-top: 2px;
  text-align: center;
  opacity: 0.8;
}

/* Kvalitet-baserte farger */
.signal-excellent {
  background: linear-gradient(135deg, #4caf50, #66bb6a);
  color: white;
  box-shadow: 0 2px 8px rgba(76, 175, 80, 0.3);
}

.signal-good {
  background: linear-gradient(135deg, #8bc34a, #9ccc65);
  color: white;
  box-shadow: 0 2px 8px rgba(139, 195, 74, 0.3);
}

.signal-fair {
  background: linear-gradient(135deg, #ff9800, #ffb74d);
  color: white;
  box-shadow: 0 2px 8px rgba(255, 152, 0, 0.3);
}

.signal-poor {
  background: linear-gradient(135deg, #f44336, #ef5350);
  color: white;
  box-shadow: 0 2px 8px rgba(244, 67, 54, 0.3);
}

.signal-none {
  background: linear-gradient(135deg, #9e9e9e, #bdbdbd);
  color: white;
  box-shadow: 0 2px 8px rgba(158, 158, 158, 0.3);
}

.signal-unknown {
  background: linear-gradient(135deg, #607d8b, #78909c);
  color: white;
  opacity: 0.7;
}

.signal-primary:hover {
  transform: translateY(-1px);
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.2);
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



/* Nye stiler for discovery og pairing */
.system-info-panel {
  background-color: #e3f2fd;
  border-radius: 8px;
  padding: 16px;
  margin-bottom: 24px;
  border-left: 4px solid #2196f3;
}

.system-info-content {
  display: flex;
  justify-content: space-between;
  align-items: center;
  flex-wrap: wrap;
  gap: 16px;
}

.system-id-display, .discovery-status {
  display: flex;
  align-items: center;
  gap: 8px;
}

.system-label, .discovery-label {
  font-weight: bold;
  color: #1976d2;
}

.system-id-value {
  font-family: monospace;
  background-color: white;
  padding: 4px 8px;
  border-radius: 4px;
  border: 1px solid #bbdefb;
}

.discovery-indicator {
  padding: 4px 8px;
  border-radius: 12px;
  font-size: 12px;
  font-weight: bold;
}

.discovery-indicator.active {
  background-color: #e8f5e9;
  color: #2e7d32;
}

.discovery-indicator:not(.active) {
  background-color: #ffebee;
  color: #c62828;
}

/* Poles grid layout */
.poles-grid {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(350px, 1fr));
  gap: 20px;
  margin-bottom: 24px;
}

/* Unassigned poles styling */
.unassigned-poles {
  margin-bottom: 32px;
}

.unassigned-poles h2 {
  color: #f57c00;
  margin-bottom: 16px;
  font-size: 20px;
}

.pole-card.unassigned {
  border-left: 4px solid #ff9800;
  background-color: #fff8e1;
}

.pole-card.unassigned .pole-status.new {
  background-color: #ffecb3;
  color: #f57c00;
}

/* Paired poles styling */
.paired-poles h2 {
  color: #388e3c;
  margin-bottom: 16px;
  font-size: 20px;
}

.pole-card.paired {
  border-left: 4px solid #4caf50;
  background-color: #f1f8e9;
}

/* Modern pole card styling */
.pole-card {
  background: linear-gradient(135deg, #ffffff 0%, #f8f9fa 100%);
  border-radius: 12px;
  box-shadow: 
    0 4px 6px rgba(0, 0, 0, 0.05),
    0 1px 3px rgba(0, 0, 0, 0.1);
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
  overflow: hidden;
  position: relative;
}

.pole-card:hover {
  transform: translateY(-2px);
  box-shadow: 
    0 8px 25px rgba(0, 0, 0, 0.1),
    0 3px 6px rgba(0, 0, 0, 0.08);
}

.pole-card.paired {
  border-left: 4px solid #4caf50;
  background: linear-gradient(135deg, #f1f8e9 0%, #ffffff 100%);
}

.pole-card.paired::before {
  content: '';
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  height: 2px;
  background: linear-gradient(90deg, #4caf50, #66bb6a);
}

/* Enhanced pole header */
.pole-title-section {
  display: flex;
  justify-content: space-between;
  align-items: flex-start;
  flex: 1;
  gap: 16px;
}

.pole-title h3 {
  margin: 0;
  font-size: 18px;
  font-weight: 600;
  color: #333;
}

.pole-subtitle {
  display: flex;
  gap: 12px;
  margin-top: 4px;
}

.mac-address {
  font-family: monospace;
  font-size: 12px;
  color: #666;
  background: #f5f5f5;
  padding: 2px 6px;
  border-radius: 3px;
}

.firmware-version {
  font-size: 12px;
  color: #0078d7;
  font-weight: 500;
}

.pole-indicators {
  display: flex;
  flex-direction: column;
  align-items: flex-end;
  gap: 8px;
}

.signal-strength {
  display: flex;
  align-items: center;
  gap: 4px;
  font-size: 12px;
  padding: 4px 8px;
  border-radius: 12px;
}

.signal-strength.signal-excellent {
  background: #e8f5e9;
  color: #2e7d32;
}

.signal-strength.signal-good {
  background: #fff3e0;
  color: #f57c00;
}

.signal-strength.signal-fair {
  background: #fff3e0;
  color: #ff9800;
}

.signal-strength.signal-poor {
  background: #ffebee;
  color: #c62828;
}

.signal-strength.signal-none {
  background: #f5f5f5;
  color: #9e9e9e;
}

.signal-icon {
  font-size: 14px;
}

.pole-status-badge {
  display: flex;
  align-items: center;
  gap: 6px;
  padding: 4px 8px;
  border-radius: 12px;
  font-size: 12px;
  font-weight: 500;
}

.pole-status-badge.connected {
  background: #e8f5e9;
  color: #2e7d32;
}

.pole-status-badge.idle {
  background: #fff3e0;
  color: #f57c00;
}

.pole-status-badge.disconnected {
  background: #ffebee;
  color: #c62828;
}

.status-dot {
  width: 8px;
  height: 8px;
  border-radius: 50%;
  background: currentColor;
}

.pole-actions-compact {
  display: flex;
  gap: 4px;
}

.icon-button {
  width: 32px;
  height: 32px;
  border: none;
  border-radius: 6px;
  background: #f5f5f5;
  color: #666;
  cursor: pointer;
  transition: all 0.2s;
  display: flex;
  align-items: center;
  justify-content: center;
}

.icon-button:hover {
  background: #e0e0e0;
  transform: scale(1.05);
}

.icon-button.identify-btn:hover {
  background: #e3f2fd;
  color: #1976d2;
}


/* Action groups styling */

.pole-actions-footer {
  display: flex;
  flex-direction: column;
  gap: 8px;
  margin-top: 16px;
}


.action-group {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 8px;
  width: 100%;
}

.action-button {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 8px 16px;
  background: white;
  border: 1px solid #e0e0e0;
  border-radius: 6px;
  font-size: 14px;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.2s ease;
  color: #555;
}

.action-button:hover {
  background: #f8f9fa;
  border-color: #0078d7;
  color: #0078d7;
}

.action-button.calibrate {
  color: #2e7d32;
}

.action-button.calibrate:hover {
  background: #f1f8e9;
  border-color: #4caf50;
}

.action-button.restart {
  color: #d32f2f;
}

.action-button.restart:hover {
  background: #ffebee;
  border-color: #f44336;
}

.action-button .icon {
  font-size: 16px;
}










.pole-card {
  background: linear-gradient(135deg, #ffffff 0%, #f8f9fa 100%);
  border-radius: 12px;
  box-shadow: 
    0 4px 6px rgba(0, 0, 0, 0.05),
    0 1px 3px rgba(0, 0, 0, 0.1);
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
  overflow: hidden;
  position: relative;
}

.pole-card:hover {
  transform: translateY(-2px);
  box-shadow: 
    0 8px 25px rgba(0, 0, 0, 0.1),
    0 3px 6px rgba(0, 0, 0, 0.08);
}

.pole-card.paired {
  background: white;
  border: 1px solid #e0e0e0;
}




/* Actions for unassigned poles */
.pole-actions-unassigned {
  display: flex;
  gap: 8px;
  margin-top: 16px;
}

.action-button.assign {
  background-color: #4caf50;
  color: white;
  flex: 1;
}

.action-button.assign:hover {
  background-color: #45a049;
}

.action-button.unpair {
  background-color: #f44336;
  color: white;
}

.action-button.unpair:hover {
  background-color: #da190b;
}

/* Instructions styling */
.instructions {
  background-color: #e8f5e9;
  border-radius: 8px;
  padding: 16px;
  margin-top: 16px;
  border-left: 4px solid #4caf50;
}

.instructions-header {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-bottom: 8px;
  color: #2e7d32;
  font-weight: bold;
}

.instructions ol {
  margin: 0;
  padding-left: 20px;
  color: #2e7d32;
}

.instructions li {
  margin-bottom: 4px;
}

/* Enhanced empty state */
.empty-state {
  text-align: center;
  padding: 48px 24px;
}

.empty-state .empty-icon {
  font-size: 64px;
  margin-bottom: 16px;
}

.empty-state h3 {
  color: #666;
  margin-bottom: 8px;
}

.empty-state p {
  color: #888;
  margin-bottom: 24px;
  max-width: 400px;
  margin-left: auto;
  margin-right: auto;
  line-height: 1.5;
}

/* Responsive adjustments */
@media (max-width: 768px) {
  .poles-grid {
    grid-template-columns: 1fr;
  }
  
  .system-info-content {
    flex-direction: column;
    align-items: flex-start;
  }
  
  .pole-actions-unassigned {
    flex-direction: column;
  }
}


/* Advanced sensor visualization */
.sensor-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 16px;
  margin-top: 16px;
}

.sensor-item {
  background: linear-gradient(135deg, #ffffff 0%, #f8f9fa 100%);
  border-radius: 8px;
  padding: 16px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.05);
  transition: all 0.3s ease;
  border: 1px solid #e0e0e0;
}

.sensor-item.active {
  border-color: #4caf50;
  box-shadow: 0 2px 8px rgba(76, 175, 80, 0.2);
}

.sensor-item:not(.enabled) {
  opacity: 0.6;
  background: linear-gradient(135deg, #f5f5f5 0%, #eeeeee 100%);
}

.sensor-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 8px;
}

.sensor-label {
  font-weight: 600;
  color: #333;
  font-size: 14px;
}

.sensor-status {
  padding: 2px 8px;
  border-radius: 12px;
  font-size: 11px;
  font-weight: bold;
  text-transform: uppercase;
}

.sensor-status.active { background: #e8f5e9; color: #2e7d32; }
.sensor-status.inactive { background: #fafafa; color: #757575; }
.sensor-status.high { background: #ffebee; color: #c62828; }
.sensor-status.medium { background: #fff3e0; color: #f57c00; }
.sensor-status.low { background: #e3f2fd; color: #1976d2; }

.sensor-value {
  font-size: 24px;
  font-weight: bold;
  color: #333;
  margin-bottom: 12px;
  font-family: 'Roboto Mono', monospace;
}

.sensor-bar-container {
  height: 8px;
  background: #e0e0e0;
  border-radius: 4px;
  position: relative;
  overflow: hidden;
  margin-bottom: 8px;
}

.sensor-bar {
  height: 100%;
  border-radius: 4px;
  transition: width 0.5s cubic-bezier(0.4, 0, 0.2, 1);
}

.sensor-bar.bar-inactive { background: #bdbdbd; }
.sensor-bar.bar-low { background: linear-gradient(90deg, #2196f3, #42a5f5); }
.sensor-bar.bar-medium { background: linear-gradient(90deg, #ff9800, #ffb74d); }
.sensor-bar.bar-high { background: linear-gradient(90deg, #f44336, #ef5350); }

.sensor-threshold {
  position: absolute;
  top: 0;
  width: 2px;
  height: 100%;
  background: #d32f2f;
  z-index: 1;
}

.sensor-threshold::before {
  content: '';
  position: absolute;
  top: -3px;
  left: -2px;
  width: 6px;
  height: 6px;
  background: #d32f2f;
  border-radius: 50%;
}

.sensor-info {
  display: flex;
  justify-content: space-between;
  align-items: center;
  font-size: 12px;
}

.threshold-text {
  color: #666;
}

.enabled-text {
  color: #4caf50;
  font-weight: 500;
}

.enabled-text.disabled {
  color: #f44336;
}

.no-sensor-data {
  text-align: center;
  padding: 32px 16px;
  color: #666;
}

.no-sensor-data .icon {
  font-size: 48px;
  margin-bottom: 16px;
  display: block;
}

.no-sensor-data p {
  margin-bottom: 8px;
  font-weight: 500;
}

.no-sensor-data small {
  color: #999;
}


/* Advanced sensor visualization */
.sensor-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 16px;
  margin-top: 16px;
}

.sensor-item {
  background: linear-gradient(135deg, #ffffff 0%, #f8f9fa 100%);
  border-radius: 8px;
  padding: 16px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.05);
  transition: all 0.3s ease;
  border: 1px solid #e0e0e0;
}

.sensor-item.active {
  border-color: #4caf50;
  box-shadow: 0 2px 8px rgba(76, 175, 80, 0.2);
}

.sensor-item:not(.enabled) {
  opacity: 0.6;
  background: linear-gradient(135deg, #f5f5f5 0%, #eeeeee 100%);
}

.sensor-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 8px;
}

.sensor-label {
  font-weight: 600;
  color: #333;
  font-size: 14px;
}

.sensor-status {
  padding: 2px 8px;
  border-radius: 12px;
  font-size: 11px;
  font-weight: bold;
  text-transform: uppercase;
}

.sensor-status.active { background: #e8f5e9; color: #2e7d32; }
.sensor-status.inactive { background: #fafafa; color: #757575; }
.sensor-status.high { background: #ffebee; color: #c62828; }
.sensor-status.medium { background: #fff3e0; color: #f57c00; }
.sensor-status.low { background: #e3f2fd; color: #1976d2; }

.sensor-value {
  font-size: 24px;
  font-weight: bold;
  color: #333;
  margin-bottom: 12px;
  font-family: 'Roboto Mono', monospace;
}

.sensor-bar-container {
  height: 8px;
  background: #e0e0e0;
  border-radius: 4px;
  position: relative;
  overflow: hidden;
  margin-bottom: 8px;
}

.sensor-bar {
  height: 100%;
  border-radius: 4px;
  transition: width 0.5s cubic-bezier(0.4, 0, 0.2, 1);
}

.sensor-bar.bar-inactive { background: #bdbdbd; }
.sensor-bar.bar-low { background: linear-gradient(90deg, #2196f3, #42a5f5); }
.sensor-bar.bar-medium { background: linear-gradient(90deg, #ff9800, #ffb74d); }
.sensor-bar.bar-high { background: linear-gradient(90deg, #f44336, #ef5350); }

.sensor-threshold {
  position: absolute;
  top: 0;
  width: 2px;
  height: 100%;
  background: #d32f2f;
  z-index: 1;
}

.sensor-threshold::before {
  content: '';
  position: absolute;
  top: -3px;
  left: -2px;
  width: 6px;
  height: 6px;
  background: #d32f2f;
  border-radius: 50%;
}

.sensor-info {
  display: flex;
  justify-content: space-between;
  align-items: center;
  font-size: 12px;
}

.threshold-text {
  color: #666;
}

.enabled-text {
  color: #4caf50;
  font-weight: 500;
}

.enabled-text.disabled {
  color: #f44336;
}

</style>