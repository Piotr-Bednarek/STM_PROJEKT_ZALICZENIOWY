import React, { useEffect, useRef } from "react";

export const Terminal = ({ logs }) => {
    const bottomRef = useRef(null);

    useEffect(() => {
        bottomRef.current?.scrollIntoView({ behavior: "smooth" });
    }, [logs]);

    return (
        <div className="terminal-container">
            <div className="terminal-header">
                <span className="terminal-title">Logi Urządzenia</span>
                <button
                    className="btn-clear"
                    onClick={() => {
                        /* Handle clear optional */
                    }}
                >
                    Wyczyść
                </button>
            </div>
            <div className="terminal-body custom-scrollbar">
                {logs.length === 0 && <div className="text-[var(--text-muted)] italic p-2">Brak logów...</div>}
                {logs.map((log, i) => (
                    <div key={i} className="log-entry">
                        <span className="timestamp">[{log.time}]</span>
                        <span className={log.type === "error" ? "log-error" : log.type === "tx" ? "log-tx" : log.type === "success" ? "log-success" : ""}>{log.msg}</span>
                    </div>
                ))}
                <div ref={bottomRef} />
            </div>
        </div>
    );
};
