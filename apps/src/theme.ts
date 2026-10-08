import { theme, type ThemeConfig } from "antd";

// The one theme for the app. styles.css reads it as --ant-* variables under the `app-theme` class;
// canvas and WebGL drawing read the same values from `tokens`.
const algorithm = theme.darkAlgorithm;
const base = theme.getDesignToken({ algorithm });

export const antdTheme: ThemeConfig = {
  algorithm,
  cssVar: { key: "app-theme" },
  components: {
    // The default track is the page colour, so the control has no visible outline on the page.
    Segmented: { trackBg: base.colorBgContainer },
  },
};

export const tokens = theme.getDesignToken(antdTheme);
