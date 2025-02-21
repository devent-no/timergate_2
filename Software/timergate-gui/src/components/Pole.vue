<script>
export default {
  data() {
    return {
      hostname: "timergate.local",
      enabled: [],
    };
  },
  methods: {
    async setBreaks() {
      var self = this;
      for (const [index, item] of this.values.entries()) {
        await fetch("http://" + this.hostname + "/api/v1/pole/break", {
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
        });
      }
    },
    async setEnabled(event) {
      console.log(event.srcElement.value)
      var self = this;
      await fetch("http://" + this.hostname + "/api/v1/pole/enabled", {
        method: "POST",
        headers: {
          Accept: "application/json",
          "Content-Type": "application/json;charset=utf-8",
          "Access-Control-Allow-Origin": "*",
        },
        body: JSON.stringify({
          sensor_nr: Number(event.srcElement.value),
          enabled: event.srcElement.checked,
          mac: self.mac,
        }),
      });
    },
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
    br_limit: {
      type: Object,
      default(rawProps) {
        return Array(7).fill(1000);
      },
    },
    offsets: {
      type: Object,
      default(rawProps) {
        return Array(7).fill(4000);
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
    show_advanced: {
      type: Boolean,
      required: true,
    }
  },
};
</script>

<template>
  <div style="height: 250px; width: 50%; float: left">
    <div style="display: flex">
      <h3 class="green centered">{{ name }} ({{ mac }})</h3>
      <button @click="setBreaks()" class="centered" style="margin-left: 10px;">Calibrate</button>
    </div>
    <div style="float: left">
      <ul>
        <li v-for="(item, index) in values" :key="index" style="display: flex; margin-bottom: 4px;">
          <input type="checkbox" v-model="enabled" :value="index" @click="setEnabled" v-if="show_advanced"/>
          <div style="width: 50px" v-if="show_advanced">{{ item }}</div>
          <div class="frame">
            <div :style="{ width: Math.round(this.br_limit[index] / 10) + 'px' }" class="break">
              <div
                :style="{
                  width: Math.round(item / 10) + 'px',
                  backgroundColor: this.broken[index],
                }"
                class="bar"
              ></div>
            </div>
          </div>
        </li>
      </ul>
    </div>
  </div>
</template>

<style scoped>
h3 {
  font-size: 1.2rem;
}
</style>
