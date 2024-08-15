<script>
export default {
  data() {
    return {
      myDate: new Date(),
    };
  },
  methods: {
    formatTime(value) {
      const time = new Date(value);
      return ( "time: "+
        ('00'+time.getHours()).slice(-2) +
        ":" +
        ('00'+time.getMinutes()).slice(-2) +
        ":" +
        ('00'+time.getSeconds()).slice(-2) +
        " ms: " +
        ('0000'+time.getMilliseconds()).slice(-4)
      ).toString();
    },
    getClass(){
      if(this.value){
        return "green"
      }
      return "red"
    }
  },
  props: {
    id: {
      type: String,
      required: true,
    },
    breakTime: {
      type: Number,
      required: true,
    },
    value: {
      type: Number,
      required: true,
    }
  },
};
</script>

<template>
  <h3 :class="getClass()">{{ formatTime(breakTime) }}</h3>
</template>

<style scoped>
h3 {
  font-size: 1.2rem;
}
</style>
