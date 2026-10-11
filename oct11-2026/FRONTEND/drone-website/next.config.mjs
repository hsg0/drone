/** @type {import('next').NextConfig} */
const nextConfig = {
  /* config options here */
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
