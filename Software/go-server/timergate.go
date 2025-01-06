package main

import (
	"bufio"
	"context"
	"encoding/json"
	"fmt"
	"io"
	"log"
	"net"
	"net/http"
	"strconv"
	"strings"
	"time"

	"nhooyr.io/websocket"
	"nhooyr.io/websocket/wsjson"
)

type Pole struct {
	Type   string `json:"type"`
	Name   string `json:"name"`
	Id     int    `json:"id"`
	Mac    string `json:"mac"`
	Values []int  `json:"values"`
	Broken []int  `json:"broken"`
}

type Break struct {
	Type  string    `json:"type"`
	Mac   string    `json:"mac"`
	Value int       `json:"value"`
	When  time.Time `json:"time"`
}

type BreakLimit struct {
	Adc_nr    int    `json:"adc"`
	Break_val int    `json:"break"`
	Mac       string `json:"mac"`
}

var poles map[string]Pole
var messageQueue = make(chan Pole)
var messageQueueBreak = make(chan Break, 2)
var sendQueues map[string]chan string

var sensor_map = [7]int{2, 6, 5, 1, 0, 3, 4}
var reversedMap = make(map[int]int)

func rootHandler(w http.ResponseWriter, r *http.Request) {
	fmt.Fprintf(w, "Start the Vue frontend and use that port")
}

func getPolesHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Access-Control-Allow-Origin", "*")
	json.NewEncoder(w).Encode(poles)
}

func setBreakLimitHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Access-Control-Allow-Origin", "*")
	w.Header().Set("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
	w.Header().Set("Access-Control-Allow-Headers", "*")

	reqBody, _ := io.ReadAll(r.Body)
	var break_limit *BreakLimit
	json.Unmarshal(reqBody, &break_limit)
	if break_limit != nil {
		adc := reversedMap[break_limit.Adc_nr]
		mac := break_limit.Mac
		sendQueues[mac] <- fmt.Sprintf("break %v = %v\n", adc, break_limit.Break_val)
	}
	json.NewEncoder(w).Encode(nil)
}

func handleWS(w http.ResponseWriter, r *http.Request) {
	fmt.Println("handleWS")

	opts := &websocket.AcceptOptions{
		OriginPatterns: []string{"localhost:5173"},
	}
	var err error
	ws, err := websocket.Accept(w, r, opts)
	if err != nil {
		log.Println("Error accepting WebSocket connection:", err)
		return
	}
	defer ws.Close(websocket.StatusInternalError, "Internal error")

	ctx, cancel := context.WithCancel(r.Context())
	defer cancel()

	go func() {
		for {
			select {
			case msg := <-messageQueue:
				err := wsjson.Write(ctx, ws, msg)
				if err != nil {
					log.Println("Error writing message from queue:", err)
					return
				}
			case <-ctx.Done():
				return
			}
		}
	}()

	go func() {
		for {
			select {
			case msg := <-messageQueueBreak:
				err := wsjson.Write(ctx, ws, msg)
				if err != nil {
					log.Println("Error writing message from BreakQueue:", err)
					return
				}
			case <-ctx.Done():
				return
			}
		}
	}()

	for {
		var msg string
		err := wsjson.Read(ctx, ws, &msg)
		if err != nil {
			log.Println("Error reading message:", err)
			break
		}
		log.Printf("Received: %s", msg)
	}

	ws.Close(websocket.StatusNormalClosure, "Normal closure")
}

func handleLine(line string, mac string) {
	if line[0:2] == "S:" {
		// Sensor value events
		adc_num, _ := strconv.Atoi(line[3:4])
		val, _ := strconv.Atoi(line[8:12])
		broken, _ := strconv.Atoi(line[17:18])
		poles[mac].Values[sensor_map[adc_num]] = val
		poles[mac].Broken[sensor_map[adc_num]] = broken
		if adc_num == 6 {
			messageQueue <- poles[mac]
		}
	} else if line[0:2] == "B:" {
		// Sensor break events
		broken, _ := strconv.Atoi(line[3:4])
		t, _ := strconv.Atoi(line[6:])
		fmt.Printf("Break sensor %s: %d @ %d\n", mac, broken, t)

		messageQueueBreak <- Break{"B", mac, broken, time.Now()}
	} else {
		fmt.Printf("'%s'\n", line)
	}
}

// Socket communication with each pole
func init_sockets() {
	ln, err := net.Listen("tcp", ":3333")
	if err != nil {
		fmt.Println(err)
		return
	}

	for {
		conn, err := ln.Accept()
		if err != nil {
			fmt.Println(err)
			continue
		}

		go handleConnection(conn)
	}
}

func handleConnection(conn net.Conn) {
	var mac string
	fmt.Println("Socket connection accepted")

	defer conn.Close()

	timeoutDuration := 5 * time.Second
	bufReader := bufio.NewReader(conn)
	bufWriter := bufio.NewWriter(conn)

	for {
		conn.SetReadDeadline(time.Now().Add(timeoutDuration))
		bytes, err := bufReader.ReadBytes('\n')
		if err != nil {
			fmt.Println(err)
			return
		}

		line := strings.TrimSuffix(string(bytes), "\n")
		if line[0:3] == "mac" {
			mac = line[6:23]
			index := len(poles)
			poles[mac] = Pole{"P",
				fmt.Sprintf("Pole %d", index+1),
				index,
				mac,
				make([]int, 7),
				make([]int, 7)}
			sendQueues[mac] = make(chan string)
			fmt.Printf("Added pole with mac %s, id %v\n", mac, index)
		} else {
			handleLine(line, mac)
		}

		queue, ok := sendQueues[mac]
		if ok {
			select {
			case msg := <-queue:
				_, err := bufWriter.Write([]byte(msg))
				if err != nil {
					log.Println("Error writing message to pole:", err)
				} else {
					err := bufWriter.Flush()
					if err != nil {
						log.Println("Error flushing buffer:", err)
					} else {
						fmt.Printf("Sent message: '%s'\n", msg)
						time.Sleep(100 * time.Millisecond)
					}
				}
			default:
				// Wait for more data
			}
		}
	}
}

func main() {
	for i, value := range sensor_map {
		reversedMap[value] = i
	}

	go init_sockets()
	poles = make(map[string]Pole)
	sendQueues = make(map[string]chan string)
	http.HandleFunc("/", rootHandler)
	http.HandleFunc("/get_poles", getPolesHandler)
	http.HandleFunc("/ws", handleWS)
	http.HandleFunc("/set_break", setBreakLimitHandler)
	log.Fatal(http.ListenAndServe(":8080", nil))
}
