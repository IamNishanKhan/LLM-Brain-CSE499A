# LLM interface for intent classification
import os
from openai import OpenAI

# Set your OpenAI API key here
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "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")

# Initialize OpenAI client
client = OpenAI(api_key=OPENAI_API_KEY)

MODEL = "gpt-4o-mini"

def call_openai_chat(prompt: str, system: str = None) -> str:
    messages = []
    if system:
        messages.append({"role": "system", "content": system})
    messages.append({"role": "user", "content": prompt})
    response = client.chat.completions.create(
        model=MODEL,
        messages=messages,
        max_tokens=4096,
        temperature=0.2
    )
    return response.choices[0].message.content.strip()

def interpret_command(command: str) -> str:
    prompt = (
        "You are an intent classifier for a home assistive robot. "
        "Given a human command, output the intent as one of: "
        "'emergency', 'scheduled_task', 'user_request', 'routine_check', or 'unknown'.\n"
        f"Command: {command}\nIntent:"
    )
    system = "You are an intent classifier for a home assistive robot."
    return call_openai_chat(prompt, system).lower() 