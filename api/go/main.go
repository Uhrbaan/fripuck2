package main

import (
	"fmt"
	"fripuck2/Fripuck2/Telemetry"
	"net"
	"time"
)

func main() {
	// The Robot's IP and Ports
	robotIP := "192.168.2.214"
	tcpAddr := robotIP + ":65430"
	udpAddrStr := ":65431"

	// 1. TCP Dial (Go initiates the connection)
	fmt.Printf("[1/2] Attempting to connect to Robot at %s...\n", tcpAddr)

	var tcpConn net.Conn
	var err error

	// Retry until the robot is ready
	for {
		tcpConn, err = net.DialTimeout("tcp", tcpAddr, 2*time.Second)
		if err == nil {
			break
		}
		fmt.Println("Robot not reachable yet, retrying...")
		time.Sleep(1 * time.Second)
	}
	defer tcpConn.Close()
	fmt.Println(">>> Connected to Robot via TCP!")

	// 2. Start UDP Listener (Wait for the stream)
	udpAddr, _ := net.ResolveUDPAddr("udp", udpAddrStr)
	udpConn, err := net.ListenUDP("udp", udpAddr)
	if err != nil {
		fmt.Printf("Failed to start UDP listener: %v\n", err)
		return
	}
	defer udpConn.Close()

	fmt.Printf("[2/2] Listening for UDP high-frequency data on %s...\n", udpAddrStr)

	// 3. Read the stream
	buffer := make([]byte, 4096)
	totalBytes := 0
	usefulBytes := 0
	droppedPackets := 0
	totalPackets := 0

	go func() {
		timer := time.NewTicker(2 * time.Second)
		startTime := time.Now()
		for {
			t := <-timer.C
			elapsed := t.Sub(startTime)
			if elapsed > 0 {
				bps := float64(totalBytes) / elapsed.Seconds()
				ubps := float64(usefulBytes) / elapsed.Seconds()

				fmt.Printf("[%.2f] Throughput: %.2fKb/s\n", elapsed.Seconds(), bps/1024)
				fmt.Printf("[%.2f] Useful throughput: %.2fKb/s\n", elapsed.Seconds(), ubps/1024)
				fmt.Printf("[%.2f] %.0f%% packets dropped.\n", elapsed.Seconds(), float32(droppedPackets)/float32(totalPackets)*100)
			}
		}
	}()

	for {
		n, remoteAddr, err := udpConn.ReadFromUDP(buffer)
		if err != nil {
			fmt.Printf("UDP Error: %v\n", err)
			continue
		}
		if remoteAddr.IP.String() != robotIP {
			fmt.Printf("Ignoring packets not coming from Robot.")
			continue
		}

		totalPackets += 1
		totalBytes += n

		if !Telemetry.BatchBufferHasIdentifier(buffer) {
			droppedPackets += 1
			continue
		}

		batch := Telemetry.GetRootAsBatch(buffer[:n], 0).UnPack()

		if len(batch.Entries) > 0 {
			for _, entry := range batch.Entries {
				// Access the Union using the Content field
				if entry.Content != nil && entry.Content.Type == Telemetry.DataInfoMessage {

					// With Object API, we can type-assert the Value directly
					infoMsg := entry.Content.Value.(*Telemetry.InfoMessageT)
					usefulBytes += len(infoMsg.Text)

					// fmt.Printf("[%d] Robot Message: %s\n", entry.Timestamp, infoMsg.Text)
				}
			}
		}
	}
}
