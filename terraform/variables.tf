variable "render_api_key" {
  description = "Render API key for authentication"
  type        = string
  sensitive   = true
}

variable "render_owner_id" {
  description = "Render owner ID (your account ID)"
  type        = string
  sensitive   = true
}

variable "openai_api_key" {
  description = "OpenAI API key for chat functionality"
  type        = string
  sensitive   = true
}

variable "qdrant_url" {
  description = "Qdrant cloud URL for vector database"
  type        = string
  sensitive   = false
}

variable "qdrant_api_key" {
  description = "Qdrant API key for authentication"
  type        = string
  sensitive   = true
}

variable "frontend_url" {
  description = "Frontend URL for CORS configuration"
  type        = string
  default     = "https://physical-ai-textbook.vercel.app"
}

variable "database_url" {
  description = "PostgreSQL database URL (Neon)"
  type        = string
  sensitive   = true
}
