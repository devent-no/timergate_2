
<script>
  export default {
    props: {
      breaks: {
        type: Object,
        default: () => ({})
      },
      passages: {
        type: Array,
        default: () => []
      }
    },
    methods: {
      formatTime(timestamp) {
        if (!timestamp) return "-";
        const date = new Date(timestamp);
        return date.toLocaleTimeString();
      },
      formatDuration(ms) {
        if (ms === null || ms === undefined) return "-";
        
        // Under 1 sekund
        if (ms < 1000) {
          return `${ms} ms`;
        }
        
        // Under 1 minutt
        if (ms < 60000) {
          const seconds = (ms / 1000).toFixed(2);
          return `${seconds} sek`;
        }
        
        // Over 1 minutt
        const minutes = Math.floor(ms / 60000);
        const seconds = ((ms % 60000) / 1000).toFixed(1);
        return `${minutes}m ${seconds}s`;
      }
    }
  };
  </script>

  <template>
    <div>
      <h1>Hendelseslogg</h1>
      
      <div class="log-container">
        <div class="log-section">
          <h2>Passeringer</h2>
          <div class="passage-list">
            <table v-if="passages.length > 0">
              <thead>
                <tr>
                  <th>Tidspunkt</th>
                  <th>Målestolpe</th>
                  <th>Sensorer</th>
                  <th>Tid siden forrige</th>
                </tr>
              </thead>
              <tbody>
                <tr v-for="(passage, index) in [...passages].reverse()" :key="index">
                  <td>{{ formatTime(passage.time) }}</td>
                  <td>{{ passage.mac }}</td>
                  <td>{{ passage.sensors }}</td>
                  <td>{{ formatDuration(passage.timeDiff) }}</td>
                </tr>
              </tbody>
            </table>
            <div v-else class="empty-state">
              Ingen passeringer registrert ennå
            </div>
          </div>
        </div>
        
        <div class="log-section">
          <h2>Sensorbrudd (rådata)</h2>
          <div class="breaks-list">
            <div v-if="Object.keys(breaks).length > 0">
              <div v-for="(brs, mac) in breaks" :key="mac" class="break-group">
                <h3>Stolpe: {{ mac }}</h3>
                <table>
                  <thead>
                    <tr>
                      <th>Tidspunkt</th>
                      <th>Sensor</th>
                      <th>Type</th>
                    </tr>
                  </thead>
                  <tbody>
                    <tr v-for="(br, idx) in [...brs].reverse().slice(0, 10)" :key="idx">
                      <td>{{ formatTime(br.time) }}</td>
                      <td>{{ br.value }}</td>
                      <td>{{ br.filtered ? 'Filtrert' : 'Rå' }}</td>
                    </tr>
                  </tbody>
                </table>
              </div>
            </div>
            <div v-else class="empty-state">
              Ingen sensorbrudd registrert ennå
            </div>
          </div>
        </div>
      </div>
    </div>
  </template>

  <style scoped>
  .log-container {
    display: flex;
    flex-direction: column;
    gap: 24px;
  }

  .log-section {
    background-color: white;
    border-radius: 8px;
    padding: 16px;
    box-shadow: 0 2px 4px rgba(0, 0, 0, 0.05);
  }

  .log-section h2 {
    margin-top: 0;
    margin-bottom: 16px;
    color: #555;
    font-size: 18px;
    border-bottom: 1px solid #eee;
    padding-bottom: 8px;
  }

  .empty-state {
    padding: 16px;
    text-align: center;
    color: #888;
    font-style: italic;
  }

  table {
    width: 100%;
    border-collapse: collapse;
  }

  th, td {
    padding: 8px 12px;
    text-align: left;
    border-bottom: 1px solid #eee;
  }

  th {
    font-weight: bold;
    color: #555;
    background-color: #f9f9f9;
  }

  .break-group {
    margin-bottom: 20px;
  }

  .break-group h3 {
    font-size: 16px;
    margin-bottom: 8px;
    color: #444;
  }
  </style>


