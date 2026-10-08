import React from "react";
import ReactDOM from "react-dom/client";
import { ConfigProvider, theme } from "antd";
import App from "./App";
import "@fontsource-variable/inter";
import "@fontsource/fira-code";

// antd needs literal colours for its dark algorithm, so these repeat the
// :root tokens in styles.css. Keep the two in step.
const antdTheme = {
  algorithm: theme.darkAlgorithm,
  token: {
    colorPrimary: "#ffa34d",
    colorInfo: "#00d9ff",
    colorLink: "#00d9ff",
    colorSuccess: "#4ade80",
    colorWarning: "#facc15",
    colorError: "#ef4444",
    colorBgBase: "#0f0f0f",
    colorBgContainer: "#1a1a1a",
    // Pop-ups (dropdowns, tooltips, the modal) cast no shadow, so a lighter fill sets them apart.
    colorBgElevated: "#252525",
    colorBorder: "#2d2d2d",
    colorBorderSecondary: "#222222",
    colorText: "#f5f5f5",
    colorTextSecondary: "#a0a0a0",
    fontFamily: '"Inter Variable", system-ui, -apple-system, "Segoe UI", sans-serif',
    fontFamilyCode: '"Fira Code", "JetBrains Mono", "Cascadia Mono", Consolas, monospace',
    fontSize: 15,
    // Corners from 4px to 8px, as --radius-* in styles.css.
    borderRadiusXS: 4,
    borderRadiusSM: 4,
    borderRadius: 6,
    borderRadiusLG: 8,
    // Flat: no shadows, and no glowing halo around a focused control.
    boxShadow: "none",
    boxShadowSecondary: "none",
    boxShadowTertiary: "none",
    controlOutlineWidth: 0,
  },
  components: {
    // Large buttons are taller, not louder: keep the body text size.
    Button: {
      primaryColor: "#1a1a1a",
      fontWeight: 600,
      contentFontSizeLG: 15,
      primaryShadow: "none",
      defaultShadow: "none",
      dangerShadow: "none",
    },
    Slider: { handleActiveOutlineColor: "transparent" },
  },
};

ReactDOM.createRoot(document.getElementById("root") as HTMLElement).render(
  <React.StrictMode>
    {/* No glowing ripple on click. */}
    <ConfigProvider theme={antdTheme} wave={{ disabled: true }}>
      <App />
    </ConfigProvider>
  </React.StrictMode>,
);
