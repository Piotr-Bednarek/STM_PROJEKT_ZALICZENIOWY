import { useCallback, useEffect, useRef, useState } from "react";

export const useSerial = () => {
    const [isConnected, setIsConnected] = useState(false);
    const [port, setPort] = useState(null);

    // Split logs into TX (sent) and RX (received) to prevent overwriting and memory issues
    const [serialHistory, setSerialHistory] = useState({ tx: [], rx: [] });

    const [dataHistory, setDataHistory] = useState([]);
    const [latestData, setLatestData] = useState({
        distance: 0,
        filtered: 0,
        error: 0,
        control: 0,
        setpoint: 150,
        freq: 0,
    });

    const readerRef = useRef(null);
    const writerRef = useRef(null);
    const streamClosedRef = useRef(null);
    const streamsRef = useRef(null);
    const keepReadingRef = useRef(false);

    // Buffer for logs to capture high-frequency data without re-rendering every frame
    const bufferRef = useRef("");
    const logBufferRef = useRef({ tx: [], rx: [] });
    const MAX_HISTORY = 100;

    // Helper to buffer logs instead of setting state directly
    const addToLogBuffer = useCallback((msg, type) => {
        const entry = { time: new Date().toLocaleTimeString(), msg, type };
        if (type === "tx") {
            logBufferRef.current.tx.push(entry);
        } else {
            logBufferRef.current.rx.push(entry);
        }
    }, []);

    // Legacy log helper for system messages (aliased to addToLogBuffer)
    const log = useCallback(
        (msg, type = "info") => {
            addToLogBuffer(msg, type);
        },
        [addToLogBuffer]
    );

    const calculateCRC8 = (text) => {
        let crc = 0x00;
        for (let i = 0; i < text.length; i++) {
            let byte = text.charCodeAt(i);
            crc ^= byte;
            for (let j = 0; j < 8; j++) {
                if (crc & 0x80) {
                    crc = (crc << 1) ^ 0x07;
                } else {
                    crc <<= 1;
                }
                crc &= 0xff;
            }
        }
        return crc;
    };

    // Refs for holding data without causing re-renders immediately
    const latestDataRef = useRef({ distance: 0, filtered: 0, error: 0, control: 0, setpoint: 150 });
    const newDataAvailableRef = useRef(false);

    const sampleCountRef = useRef(0);
    const lastFreqUpdateRef = useRef(Date.now());

    // Throttled update loop (Runs at ~20FPS)
    useEffect(() => {
        const interval = setInterval(() => {
            const now = Date.now();

            // 1. Calculate Frequency every ~1s
            if (now - lastFreqUpdateRef.current >= 1000) {
                const hz = sampleCountRef.current;
                latestDataRef.current.freq = hz;
                sampleCountRef.current = 0;
                lastFreqUpdateRef.current = now;
                newDataAvailableRef.current = true;
            }

            // 2. Flush Log Buffer to State
            const hasTx = logBufferRef.current.tx.length > 0;
            const hasRx = logBufferRef.current.rx.length > 0;

            if (hasTx || hasRx) {
                setSerialHistory((prev) => {
                    let newTx = prev.tx;
                    let newRx = prev.rx;

                    if (hasTx) {
                        // Keep last 100 TX commands (User wants them to stay)
                        newTx = [...prev.tx, ...logBufferRef.current.tx].slice(-100);
                        logBufferRef.current.tx = [];
                    }
                    if (hasRx) {
                        // Keep last 50 RX frames (High speed data)
                        newRx = [...prev.rx, ...logBufferRef.current.rx].slice(-50);
                        logBufferRef.current.rx = [];
                    }
                    return { tx: newTx, rx: newRx };
                });
            }

            // 3. Update Metrics Data
            if (newDataAvailableRef.current) {
                const snap = { ...latestDataRef.current };
                setLatestData(snap);

                setDataHistory((prev) => {
                    const newHist = [...prev, { ...snap, timestamp: Date.now() }];
                    if (newHist.length > MAX_HISTORY) return newHist.slice(newHist.length - MAX_HISTORY);
                    return newHist;
                });

                newDataAvailableRef.current = false;
            }
        }, 100); // Update GUI at 10 FPS to save memory

        return () => clearInterval(interval);
    }, []);

    const processLine = useCallback(
        (line) => {
            let valid = false;
            let payload = line;

            if (line.includes(";C:")) {
                const parts = line.split(";C:");
                if (parts.length === 2) {
                    const dataPart = parts[0];
                    const crcRecv = parseInt(parts[1], 16);
                    const crcCalc = calculateCRC8(dataPart);
                    if (crcCalc === crcRecv) {
                        valid = true;
                        payload = dataPart;
                        // Log valid received frame
                        log(`📥 RX: ${line}`, "info");
                    } else {
                        // Log CRC error
                        log(`❌ CRC: ${line} (calc:${crcCalc.toString(16).toUpperCase()}, recv:${crcRecv.toString(16).toUpperCase()})`, "error");
                    }
                }
            } else {
                // Log frame without CRC (if any)
                log(`📥 RX (no CRC): ${line}`, "info");
            }

            if (valid) {
                // Update Ref directly (fast, no render)
                sampleCountRef.current++;
                const segments = payload.split(";");
                const currentFnData = latestDataRef.current; // access ref

                segments.forEach((seg) => {
                    if (seg.startsWith("D:")) {
                        currentFnData.distance = Math.max(0, Math.min(parseFloat(seg.substring(2)), 290));
                    } else if (seg.startsWith("F:")) {
                        currentFnData.filtered = parseFloat(seg.substring(2));
                    } else if (seg.startsWith("E:")) {
                        currentFnData.error = parseFloat(seg.substring(2));
                    } else if (seg.startsWith("A:")) {
                        currentFnData.control = parseFloat(seg.substring(2));
                    }
                });
                newDataAvailableRef.current = true;
            }
        },
        [log]
    );

    const connect = async () => {
        if (!navigator.serial) {
            alert("API Web Serial nie jest obsługiwane w tej przeglądarce.");
            return;
        }

        try {
            const selectedPort = await navigator.serial.requestPort();
            await selectedPort.open({ baudRate: 115200 }); // Default match Python app

            setPort(selectedPort);
            setIsConnected(true);
            log("Połączono z urządzeniem", "success");

            // Setup Read Loop
            const textDecoder = new TextDecoderStream();
            const readableStreamClosed = selectedPort.readable.pipeTo(textDecoder.writable);
            const reader = textDecoder.readable.getReader();

            const textEncoder = new TextEncoderStream();
            const writableStreamClosed = textEncoder.readable.pipeTo(selectedPort.writable);
            const writer = textEncoder.writable.getWriter();

            readerRef.current = reader;
            writerRef.current = writer;
            keepReadingRef.current = true;
            streamClosedRef.current = { readable: readableStreamClosed, writable: writableStreamClosed };
            streamsRef.current = { readable: textDecoder.readable, writable: textEncoder.writable };

            readLoop(reader);
        } catch (err) {
            log(`Błąd połączenia: ${err.message}`, "error");
            console.error(err);
        }
    };

    const readLoop = async (reader) => {
        try {
            while (keepReadingRef.current) {
                const { value, done } = await reader.read();
                if (done) break;
                if (value) {
                    bufferRef.current += value;
                    // Split by newline
                    const lines = bufferRef.current.split("\n");
                    // Process all complete lines
                    while (lines.length > 1) {
                        const line = lines.shift().trim();
                        if (line) processLine(line);
                    }
                    // Keep the last part (incomplete line)
                    bufferRef.current = lines[0];
                }
            }
        } catch (err) {
            log(`Utracono połączenie: ${err.message}`, "error");
            // Don't call disconnect() directly here to avoid race conditions.
            // The reader lock release below will allow cleanup.
            // We can trigger a state update that causes cleanup or just let user click disconnect.
            // But for auto-disconnect, we need to be careful.
            // Best: Call disconnect but ensure it handles "released" state.
            disconnect();
        } finally {
            try {
                reader.releaseLock();
            } catch (e) {
                console.error(e);
            }
        }
    };

    const disconnect = async () => {
        keepReadingRef.current = false;

        // 1. Close Reader
        if (readerRef.current) {
            try {
                await readerRef.current.cancel();
            } catch (e) {
                // Ignore "reader has been released" error
                if (!e.message.includes("released")) {
                    console.error("Error cancelling reader", e);
                }
            }
            readerRef.current = null;
        }

        // 1b. Ensure Readable Stream is definitely cancelled if reader was already released
        if (streamsRef.current?.readable) {
            try {
                const r = streamsRef.current.readable.getReader();
                await r.cancel();
                r.releaseLock();
            } catch (e) {
                // Ignore if already locked/closed
            }
        }

        // 2. Wait for Readable Stream Pipe to close
        if (streamClosedRef.current?.readable) {
            try {
                await streamClosedRef.current.readable.catch(() => {});
            } catch (e) {
                console.error(e);
            }
        }

        // 3. Close Writer
        if (writerRef.current) {
            try {
                await writerRef.current.close();
            } catch (e) {
                // If close fails (e.g. stream error), try abort
                try {
                    await writerRef.current.abort();
                } catch (e2) {}
            }
            writerRef.current = null;
        }

        // 4. Wait for Writable Stream Pipe to close
        if (streamClosedRef.current?.writable) {
            try {
                await streamClosedRef.current.writable.catch(() => {});
            } catch (e) {
                console.error(e);
            }
        }

        // 5. Close Port
        if (port) {
            try {
                await port.close();
            } catch (e) {
                console.error("Error closing port", e);
            }
        }

        setIsConnected(false);
        setPort(null);
        streamClosedRef.current = null;
        streamsRef.current = null;
        log("Rozłączono", "info");
    };

    const sendCommand = async (cmd) => {
        if (!writerRef.current) return;
        try {
            const crc = calculateCRC8(cmd);
            const msg = `${cmd};C:${crc.toString(16).toUpperCase().padStart(2, "0")}\n`;
            await writerRef.current.write(msg);
            addToLogBuffer(`${msg.trim()}`, "tx"); // Use buffer for TX logs too
        } catch (err) {
            log(`Błąd wysyłania: ${err}`, "error");
        }
    };

    const sendSetpoint = (val) => {
        // Format: S:150.0
        sendCommand(`S:${val.toFixed(1)}`);
        // Update both Reace state (optimistic) AND Ref (so loop doesn't overwrite it with 0)
        latestDataRef.current.setpoint = val;
        setLatestData((prev) => ({ ...prev, setpoint: val }));
    };

    const sendPid = async (kp, ki, kd) => {
        const delay = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

        // Send P
        await sendCommand(`P:${kp.toFixed(4)}`);
        await delay(200);

        // Send I
        await sendCommand(`I:${ki.toFixed(5)}`);
        await delay(200);

        // Send D
        await sendCommand(`D:${kd.toFixed(1)}`);
    };

    const sendCalibration = async (index, rawValue, targetValue) => {
        // Format: CAL0:123.4,0.0
        await sendCommand(`CAL${index}:${rawValue.toFixed(1)},${targetValue.toFixed(1)}`);
    };

    return {
        isConnected,
        connect,
        disconnect,
        logs: serialHistory, // Export split history object {tx, rx}
        latestData,
        dataHistory,
        sendSetpoint,
        sendPid,
        sendCalibration,
    };
};
