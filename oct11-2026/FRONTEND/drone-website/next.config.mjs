/** @type {import('next').NextConfig} */
const nextConfig = {
  /* config options here */
  // The Pi serves this on 0.0.0.0, so Next calls 127.0.0.1 and the Pi address
  // other hosts and blocks its own dev files. Without these the page never starts.
  allowedDevOrigins: ["127.0.0.1", "localhost", "192.168.0.138", "192.168.0.130"],
  cacheComponents: true,
  partialPrefetching: true,
  reactCompiler: true,
  turbopack: {
    rules: {
      "*.css": {
        loaders: ["@tailwindcss/turbopack"],
        as: "*.css",
      },
    },
  },
  async rewrites() {
    const backendTarget = process.env.BACKEND_URL || "http://127.0.0.1:8000";
    return [
      {
        source: "/backend/:path*",
        destination: `${backendTarget}/:path*`,
      },
    ];
  },
};

export default nextConfig;
