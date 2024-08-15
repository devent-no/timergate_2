<script>
export default {
  data() {
    return {      
      hostname: "timergate.local"
    };
  },
  methods: {
    async setBreaks() {
      var self = this
      for (const [index, item] of this.values.entries()){
        await fetch("http://"+this.hostname+"/api/v1/pole/break", {
          method: "POST",
          headers: {
            Accept: "application/json",
            "Content-Type": "application/json;charset=utf-8",
            "Access-Control-Allow-Origin": "*",
          },
          body: JSON.stringify({
            adc: index,
            break: Math.max(item - 300, 100),
            mac: self.mac,
          }),
        })
      }
    }
  },
  props: {
    name: {
      type: String,
      required: true,
    },
    values: {
      type: Object,
      default(rawProps) {
        return Array(7).fill(4096);
      },
    },
    broken: {
      type: Object,
      default(rawProps) {
        return Array(7).fill("#f87979");
      },
    },
    mac: {
      type: String,
      required: true,
    },
  },
  computed: {
    chartData() {
      return {
        labels: ["ADC 0", "ADC 1", "ADC 2", "ADC 3", "ADC 4", "ADC 5", "ADC 6"],
        datasets: [
          {
            label: "ADC values",
            backgroundColor: this.broken,
            data: this.values,
          },
        ],
      };
    },
  },
};
</script>

<template>
  <div style="height: 250px; width: 50%; float: left">
    <div style="display: flex;">
      <h3 class="green centered">{{ name }} ({{mac}})</h3>
      <button @click="setBreaks()" class="centered">Calibrate</button>
    </div>
    <div style="float: left;">
      <ul>
        <li v-for="(item, index) in values" :key="index" style="display: flex">
          <div style="width: 50px;">{{ item }}</div>
          <div :style="{'width': Math.round(item/10)+'px', backgroundColor: this.broken[index] } " class="bar"></div>
        </li>
      </ul>
    </div>
  </div>
</template>

<style scoped>
h3 {
  font-size: 1.2rem;
}

.greetings h1,
.greetings h3 {
  text-align: center;
}
</style>
