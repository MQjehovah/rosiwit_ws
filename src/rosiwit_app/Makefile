# ============================================================
# rosiwit_app - Makefile for Build & Deploy Automation
# ============================================================
# Usage:
#   make build          - Build rosiwit_app package
#   make build-all      - Build entire workspace
#   make docker-build   - Build Docker runtime image
#   make docker-dev     - Build Docker dev image
#   make docker-up      - Start full stack (Docker)
#   make docker-down    - Stop all containers
#   make test           - Run Python tests
#   make lint           - Run code quality checks
#   make clean          - Clean build artifacts
# ============================================================

SHELL := /bin/bash
ROS_WS := $(shell cd ../../.. && pwd)
DOCKER_DIR := docker

# Colors for output
GREEN  := \033[0;32m
YELLOW := \033[0;33m
RED    := \033[0;31m
NC     := \033[0m

.PHONY: help build build-all test lint clean \
        docker-build docker-dev docker-up docker-down docker-logs \
        docker-shell docker-test

help: ## Show this help
	@echo "rosiwit_app - Build & Deploy Commands"
	@echo "====================================="
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  $(GREEN)%-20s$(NC) %s\n", $$1, $$2}'

# ==================== Native Build ====================
build: ## Build rosiwit_app only
	@echo "$(GREEN)Building rosiwit_app...$(NC)"
	@source /opt/ros/humble/setup.bash && \
		cd $(ROS_WS) && \
		colcon build --packages-select rosiwit_app --symlink-install \
			--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

build-all: ## Build entire workspace
	@echo "$(GREEN)Building entire workspace...$(NC)"
	@source /opt/ros/humble/setup.bash && \
		cd $(ROS_WS) && \
		colcon build --symlink-install \
			--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# ==================== Testing ====================
test: ## Run Python unit tests
	@echo "$(GREEN)Running tests...$(NC)"
	@cd $(ROS_WS)/src/rosiwit_app && \
		python3 -m pytest tests/ -v --tb=short

test-cov: ## Run tests with coverage
	@echo "$(GREEN)Running tests with coverage...$(NC)"
	@cd $(ROS_WS)/src/rosiwit_app && \
		python3 -m pytest tests/ -v --tb=short \
			--cov=rosiwit_app --cov-report=term-missing

# ==================== Linting ====================
lint: ## Run code quality checks
	@echo "$(GREEN)Running linting...$(NC)"
	@cd $(ROS_WS)/src/rosiwit_app && \
		echo "=== flake8 ===" && \
		python3 -m flake8 rosiwit_app/ --max-line-length=120 --count --statistics && \
		echo "=== pydocstyle ===" && \
		python3 -m pydocstyle rosiwit_app/ --count || true

# ==================== Docker ====================
docker-build: ## Build Docker runtime image
	@echo "$(GREEN)Building Docker runtime image...$(NC)"
	@cd $(DOCKER_DIR) && \
		docker compose build sim-slam-nav

docker-dev: ## Build Docker dev image
	@echo "$(GREEN)Building Docker dev image...$(NC)"
	@cd $(DOCKER_DIR) && \
		docker compose build dev

docker-up: ## Start full stack with Docker
	@echo "$(GREEN)Starting full stack...$(NC)"
	@xhost +local:docker 2>/dev/null || true
	@cd $(DOCKER_DIR) && \
		docker compose up -d sim-slam-nav

docker-up-app: ## Start app only with Docker
	@echo "$(GREEN)Starting app only...$(NC)"
	@cd $(DOCKER_DIR) && \
		docker compose up -d app-only

docker-down: ## Stop all Docker containers
	@echo "$(YELLOW)Stopping containers...$(NC)"
	@cd $(DOCKER_DIR) && \
		docker compose down

docker-logs: ## Follow Docker logs
	@cd $(DOCKER_DIR) && \
		docker compose logs -f

docker-shell: ## Open shell in dev container
	@echo "$(GREEN)Opening dev shell...$(NC)"
	@cd $(DOCKER_DIR) && \
		docker compose run --rm dev

docker-test: ## Run tests in Docker
	@echo "$(GREEN)Running tests in Docker...$(NC)"
	@cd $(DOCKER_DIR) && \
		docker compose run --rm test

# ==================== Cleanup ====================
clean: ## Clean build artifacts
	@echo "$(YELLOW)Cleaning build artifacts...$(NC)"
	@rm -rf $(ROS_WS)/build/rosiwit_app
	@rm -rf $(ROS_WS)/install/rosiwit_app
	@rm -rf $(ROS_WS)/log/latest_build/rosiwit_app
	@find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	@find . -type f -name "*.pyc" -delete 2>/dev/null || true
	@echo "$(GREEN)Clean complete.$(NC)"
