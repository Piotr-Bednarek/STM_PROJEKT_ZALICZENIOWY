import { useCallback, useEffect, useRef, useState } from "react";

export const useSerial = () => {
    const [isConnected, setIsConnected] = useState(false);
    const [port, setPort] = useState(null);
    const [logs, setLogs] = useState([]);
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
    const streamsRef = useRef(null); // Keep references to the streams themselves
    const keepReadingRef = useRef(false);

    // Buffer for incoming chunks
    const bufferRef = useRef("");
    const MAX_HISTORY = 100; // Reduced from 200 to save memory

    const log = useCallback((msg, type = "info") => {
        setLogs((prev) => [...prev.slice(-49), { time: new Date().toLocaleTimeString(), msg, type }]); // Keep last 50 only
    }, []);

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

    // Throttled update loop
    useEffect(() => {
        const interval = setInterval(() => {
            const now = Date.now();

            // Calculate Frequency every 1s (approx)
            if (now - lastFreqUpdateRef.current >= 1000) {
                const hz = sampleCountRef.current;
                latestDataRef.current.freq = hz;
                sampleCountRef.current = 0;
                lastFreqUpdateRef.current = now;
                newDataAvailableRef.current = true; // Force update to show new freq
            }

            if (newDataAvailableRef.current) {
                // Clone stats to trigger re-render
                const snap = { ...latestDataRef.current };
                setLatestData(snap);

                setDataHistory((prev) => {
                    const newHist = [...prev, { ...snap, timestamp: Date.now() }];
                    if (newHist.length > MAX_HISTORY) return newHist.slice(newHist.length - MAX_HISTORY);
                    return newHist;
                });

                newDataAvailableRef.current = false;
            }
        }, 50); // Update GUI at 20 FPS max

        return () => clearInterval(interval);
    }, []);

    const processLine = useCallback((line) => {
        // ... (Parsing logic same as before, but updates ref)
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
                }
            }
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
            // We assume Setpoint is handled via sendSetpoint optimistically,
            // OR if STM sends it back, update it here too.
            // For now, keep existing setpoint.

            newDataAvailableRef.current = true;
        }
    }, []);

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
            log(`Wysłano: ${msg.trim()}`, "tx");
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

    return {
        isConnected,
        connect,
        disconnect,
        logs,
        latestData,
        dataHistory,
        sendSetpoint,
        sendPid,
    };
};
