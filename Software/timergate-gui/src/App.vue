<script>
import BreakItem from "./components/BreakItem.vue";
import Pole from "./components/Pole.vue";

export default {
  components: {
    BreakItem,
    Pole,
  },
  data() {
    return {
      count: 0,
      poles: [],
      lookup: new Map(),
      breaks: {},
      socket: null,
      socket_ready: false,
      hostname: "timergate.local",
      interval: null,
      time: null
    };

  },
  methods: {
    init() {
      this.socket = new WebSocket("ws://" + this.hostname + "/ws");
      this.socket.onopen = this.onSocketOpen;
      this.socket.onmessage = this.onSocketMessage;
      this.socket.onerror = this.onSockerError;
    },
    clearTimes() {
      this.breaks = {};
    },
    async syncTime() {
        await fetch("http://"+this.hostname+"/api/v1/time/sync", {
          method: "POST",
          headers: {
            Accept: "application/json",
            "Content-Type": "application/json;charset=utf-8",
            "Access-Control-Allow-Origin": "*",
          },
          body: JSON.stringify({
            timestamp: Math.floor(Date.now() / 1000)
          }),
        })
    },
    onSocketOpen(evt) {
      this.socket_ready = true;
    },
    onSocketMessage(evt) {
      var received = JSON.parse(evt.data);
      if (!this.lookup.has(received.M)) {
        this.lookup.set(received.M, this.poles.length);
      }
      const id = this.lookup.get(received.M);
      if (received.K == 0) {
        this.poles[id] = {
          name: "Pole " + id,
          id: id,
          mac: received.M,
          values: received.V,
          broken: received.B,
        };
        this.poles[id].broken = this.poles[id].broken.map((x) =>
          x == 1 ? "#f87979" : "#087979"
        );
      } else if (received.K == 1) {
        if (!(received.M in this.breaks)) {
          this.breaks[received.M] = [];
        }
        const br = { mac: received.M, time: (received.T*1000 + Math.round(received.U/1000)), value: received.B };
        this.breaks[received.M].push(br);
      }
    },
    onSockerError(evt) {
      this.socket_ready = false;
      console.log("On Error");
    },
  },
  mounted() {
    this.init();
  },
  beforeDestroy() {
    clearInterval(this.interval)
  },
  created() {
    this.interval = setInterval(() => {
      this.time = Intl.DateTimeFormat("NO", {
        hour: 'numeric',
        minute: 'numeric',
        second: 'numeric'
      }).format()
    }, 1000)
  }
};
</script>

<template>
  <main>
    <h1>Timergate</h1>
      <h3>{{ time }}</h3>
    <button @click="syncTime()">Sync Time</button>
    <h1>Poles</h1>
    <div style="width: 100%; height: 250px">
      <Pole
        v-for="pole in poles"
        :key="pole.Id"
        :name="pole.name"
        :mac="pole.mac"
        :values="pole.values"
        :broken="pole.broken"
      />
    </div>
    <div>
      <h1>Breaks</h1>
      <button @click="clearTimes()">Clear</button>
    </div>
    <div
      v-for="(brs, key) in breaks"
      :key="key"
      style="width: 500px; height: 500px; float: left"
    >
      <ul v-for="br in brs.slice().reverse()" :key="br.id">
        <BreakItem
          :id="br.mac"
          :breakTime="br.time"
          :value="br.value"
        ></BreakItem>
      </ul>
    </div>
  </main>
</template>

<style scoped>
header {
  line-height: 1.5;
}
</style>
