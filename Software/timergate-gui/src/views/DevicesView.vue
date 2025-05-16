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
    }
  }
}
</script>

<template>
    <div>
      <h1>Enheter</h1>
      <p>Her kommer enhetsadministrasjon</p>
    </div>
</template>